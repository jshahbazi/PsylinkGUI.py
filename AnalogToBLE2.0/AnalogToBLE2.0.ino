// Dependencies: (Install in ArduinoIDE by clicking on Tools -> Manage Libraries...)
// 1. ArduinoBLE
// 2. Arduino_LSM9DS1

// Flash storage courtesy of https://github.com/petewarden/arduino_nano_ble_write_flash

// Feature flags
#define SEND_METRICS false

#include <ArduinoBLE.h>       // Bluetooth Low Energy
#include <Arduino_LSM9DS1.h>  // Inertial Measurement Unit
#include "FlashIAPBlockDevice.h" // Flash Storage
#if USE_INTERRUPT_TIMER == true
#include "NRF52_MBED_TimerInterrupt.h"
#endif

// Configuration
#define SAMPLE_RATE 500 // samples per second
#define BLE_NOTIFY_RATE 20 // updates per second
#define BLE_CONNECTION_INTERVAL_MIN 8 // in steps of 1.25ms
#define BLE_CONNECTION_INTERVAL_MAX 8 // in steps of 1.25ms
#define JUMPER_PIN_TO_DISABLE_IMU D2
#define CHANNELS 8
#define BUFFERS 2 // multiple buffers help with concurrency issues, if needed
#define METADATA_BYTES 8
// Metadata format:
// Bytes   |1   |2                            |3-5             |6-8                 |9-11               |12-20
// Bits    |all |1-4           |5-8           |all             |all                 |all                |all
// Content |Tick|MinSampleDelay|MaxSampleDelay|gyroscope(x,y,z)|Accelerometer(x,y,z)|Magnetometer(x,y,z)|yaw,pitch,roll
// NOTE: MinSampleDelay and MaxSampleDelay will be 0xF when SEND_METRICS is false.

// The DELAY_PARAMs are from a logarithmic regression (f(x)=A+B*log(x)) with
// f(1000)=2 and f(500000)=14, so we can map ranges from <1000us to >500000us to 4 bits.
// Keep this in sync with python/psylink/protocol.py.
#define DELAY_PARAM_A -11.3384217
#define DELAY_PARAM_B 1.93093431
#define COMPRESS_DELAY(x) ((int) min(15, max(1, round(DELAY_PARAM_A + DELAY_PARAM_B * log(x)))))

// Constants
const int SAMPLE_INTERVAL_uS = 1000000 / SAMPLE_RATE;
const int BLE_NOTIFY_INTERVAL_MS = 1000 / BLE_NOTIFY_RATE;
const int SAMPLES_PER_NOTIFY = SAMPLE_RATE / BLE_NOTIFY_RATE;
// Ensure that BLE_CHARACTERISTICS_SIZE does not exceed BLE characteristic length limit of 512 bytes
const int BLE_CHARACTERISTIC_SIZE = METADATA_BYTES + CHANNELS * SAMPLES_PER_NOTIFY;
const int NO_BUFFER = -1;

#if USE_INTERRUPT_TIMER == true
NRF52_MBED_Timer samplingTimer(NRF_TIMER_3);
#endif
BLEDevice connectedDevice;
BLEService sensorService("0a3d3fd8-2f1c-46fd-bf46-eaef2fda91e4");
BLEStringCharacteristic sensorCharacteristic("0a3d3fd8-2f1c-46fd-bf46-eaef2fda91e5", BLERead, BLE_CHARACTERISTIC_SIZE);
BLEIntCharacteristic channelCountCharacteristic("0a3d3fd8-2f1c-46fd-bf46-eaef2fda91e6", BLERead);

float yaw = 0.0, pitch = 0.0, roll = 0.0;
struct principalAxes {
  float yaw;
  float pitch;
  float roll;
};
struct principalAxes dataPacketAxes;
BLECharacteristic axesCharacteristic("0a3d3fd8-2f1c-46fd-bf46-eaef2fda91a1", BLERead | BLENotify, sizeof(dataPacketAxes));

struct imuData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;  
  float mx;
  float my;
  float mz;
};

struct imuData dataPacketIMU;
BLECharacteristic imuCharacteristic("0a3d3fd8-2f1c-46fd-bf46-eaef2fda91a2", BLERead | BLENotify, sizeof(dataPacketIMU));

volatile bool doSampling = true;
volatile int sendBuffer = NO_BUFFER;
int samples[BUFFERS][CHANNELS][SAMPLES_PER_NOTIFY] = {0};
int currentSample = 0;
int currentBuffer = 0;
unsigned char tick = 1;
char bleString[BLE_CHARACTERISTIC_SIZE] = {0};
bool bleConnected = false;

// Metrics
#if SEND_METRICS == true
volatile unsigned long minSampleDelay, maxSampleDelay, lastSampleMicroSeconds = 0;
#endif

// Flash storage prep
constexpr int kFlashBlockSize = 4096;
#define ROUND_UP(val, block_size) ((((val) + ((block_size) - 1)) / (block_size)) * (block_size))
constexpr int kFlashBufferSize = ROUND_UP(64 * 1024, kFlashBlockSize);
alignas(kFlashBlockSize) const uint8_t flash_buffer[kFlashBufferSize] = {};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////
////////////////////////////  IMU Code
////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct bias_values
{
  int calibrated;
  float gyroBiasX;
  float gyroBiasY;
  float gyroBiasZ;
  float accelBiasX;
  float accelBiasY;
  float accelBiasZ;
  float magBiasX;
  float magBiasY;
  float magBiasZ;
};

bias_values *biasData;


// IMU Calibration //////////////////////////////////////////////////////////////////////////////

#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should be 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28

// Using the LSM9DS1+MS5611 Teensy 3.1 Add-On shield, ADO is set to 1
// Seven-bit device address of accel/gyro is 110101 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define LSM9DS1XG_ADDRESS 0x6B  //  Device address when ADO = 1
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer
#define MS5611_ADDRESS    0x77  //  Address of altimeter
#else
#define LSM9DS1XG_ADDRESS 0x6A   //  Device address when ADO = 0
#define LSM9DS1M_ADDRESS  0x1D   //  Address of magnetometer
#define MS5611_ADDRESS    0x77   //  Address of altimeter
#endif

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_16G,
  AFS_4G,
  AFS_8G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_10Hz,
  AODR_50Hz,
  AODR_119Hz,
  AODR_238Hz,
  AODR_476Hz,
  AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
  ABW_408Hz = 0,
  ABW_211Hz,
  ABW_105Hz,
  ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
  GODR_PowerDown = 0,
  GODR_14_9Hz,
  GODR_59_5Hz,
  GODR_119Hz,
  GODR_238Hz,
  GODR_476Hz,
  GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
  GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
  GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
  GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale {  // set of allowable mag full scale settings
  MFS_4G = 0,
  MFS_8G,
  MFS_12G,
  MFS_16G
};

enum Mmode {
  MMode_LowPower = 0,
  MMode_MedPerformance,
  MMode_HighPerformance,
  MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_80Hz
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor full scale
uint8_t OSR = ADC_4096;      // set pressure amd temperature oversample rate
uint8_t Gscale = GFS_245DPS; // gyro full scale
uint8_t Godr = GODR_238Hz;   // gyro data sample rate
uint8_t Gbw = GBW_med;       // gyro data bandwidth
uint8_t Ascale = AFS_2G;     // accel full scale
uint8_t Aodr = AODR_238Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_4G;     // mag full scale
uint8_t Modr = MODR_10Hz;    // mag data sample rate
uint8_t Mmode = MMode_HighPerformance;  // magnetometer operation mode
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int myLed  = 13;

uint16_t Pcal[8];         // calibration constants from MS5611 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t _D1 = 0, _D2 = 0;  // raw MS5611 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5611 pressures sensor pressure and temperature

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case MFS_4G:
      mRes = 4.0 / 32768.0;
      break;
    case MFS_8G:
      mRes = 8.0 / 32768.0;
      break;
    case MFS_12G:
      mRes = 12.0 / 32768.0;
      break;
    case MFS_16G:
      mRes = 16.0 / 32768.0;
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11).
    case GFS_245DPS:
      gRes = 245.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11).
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_TEMP_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}


void initLSM9DS1()
{
  // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 | Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
}


void selftestLSM9DS1()
{
  float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
  float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  accelgyrocalLSM9DS1(gyro_noST, accel_noST);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
  accelgyrocalLSM9DS1(gyro_ST, accel_ST);

  float gyrodx = (gyro_ST[0] - gyro_noST[0]);
  float gyrody = (gyro_ST[1] - gyro_noST[1]);
  float gyrodz = (gyro_ST[2] - gyro_noST[2]);

  if(Serial.available()){
    Serial.println("Gyro self-test results: ");
    Serial.print("x-axis = "); Serial.print(gyrodx); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
    Serial.print("y-axis = "); Serial.print(gyrody); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
    Serial.print("z-axis = "); Serial.print(gyrodz); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
  }
  float accdx = 1000.*(accel_ST[0] - accel_noST[0]);
  float accdy = 1000.*(accel_ST[1] - accel_noST[1]);
  float accdz = 1000.*(accel_ST[2] - accel_noST[2]);

  if(Serial.available()){
    Serial.println("Accelerometer self-test results: ");
    Serial.print("x-axis = "); Serial.print(accdx); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
    Serial.print("y-axis = "); Serial.print(accdy); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
    Serial.print("z-axis = "); Serial.print(accdz); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
  }
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  delay(200);
}
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalLSM9DS1(float * dest1, float * dest2)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;

  // enable the 3-axes of the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
  // configure the gyroscope
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
  delay(200);
  // enable the three axes of the accelerometer
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
  // configure the accelerometer-specify bandwidth selection with Abw
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 | Abw);
  delay(200);
  // enable block data update, allow auto-increment during multiple byte read
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);

  // First get gyro bias
  byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for (ii = 0; ii < samples ; ii++) {           // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1];
    gyro_bias[2] += (int32_t) gyro_temp[2];
  }

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples;
  gyro_bias[2] /= samples;

  dest1[0] = (float)gyro_bias[0] * gRes; // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1] * gRes;
  dest1[2] = (float)gyro_bias[2] * gRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode

  // now get the accelerometer bias
  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO
  delay(50);                                                       // Wait for change to take effect
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

  for (ii = 0; ii < samples ; ii++) {           // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
  }

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples;
  accel_bias[2] /= samples;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) (1.0 / aRes); // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) (1.0 / aRes);
  }

  dest2[0] = (float)accel_bias[0] * aRes; // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1] * aRes;
  dest2[2] = (float)accel_bias[2] * aRes;

  c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO
  delay(50);
  writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void magcalLSM9DS1(float * dest1)
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  // configure the magnetometer-enable temperature compensation of mag data
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  sample_count = 128;
  for (ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
  }

  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes;
  dest1[2] = (float) mag_bias[2] * mRes;

  //write biases to accelerometermagnetometer offset registers as counts);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);

  Serial.println("Mag Calibration done!");
}

// I2C read/write functions for the LSM9DS1and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(subAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire1.beginTransmission(address);         // Initialize the Tx buffer
  Wire1.write(subAddress);                  // Put slave register address in Tx buffer
  //  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  //  Wire.requestFrom(address, 1);  // Read one byte from slave register address
  Wire1.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
  data = Wire1.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(subAddress);            // Put slave register address in Tx buffer
  //  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire1.requestFrom(address, count);  // Read bytes from slave register address
  //        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  while (Wire1.available()) {
    dest[i++] = Wire1.read();
  }         // Put read results in the Rx buffer
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  Serial.begin(9600);
  int serialStart = millis();
  bool serialFlag = true;
  while (Serial.available()<=0){
    if ((millis() - serialStart) >= 5000){
      serialFlag = false;
      break;
    }
    delay(10);
  }

  analogReadResolution(12);
  //  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  //  pinMode(LED_PWR, OUTPUT);
  pinMode(myLed, OUTPUT);
  pinMode(JUMPER_PIN_TO_DISABLE_IMU, INPUT_PULLUP);  // HIGH by default. IMU disabled on LOW.
  digitalWrite(LEDR, HIGH); // The LED is LOW-activated, let's turn it off.
  digitalWrite(LEDG, HIGH); // The LED is LOW-activated, let's turn it off.
  //digitalWrite(LEDB, HIGH); // The LED is LOW-activated, let's turn it off.
  analogWrite(LEDB, 255); // The LED is LOW-activated, let's turn it off.
  if (!BLE.begin()) {
    digitalWrite(LEDR, LOW); // Turn on red LED
    while (1);
  }
#if USE_INTERRUPT_TIMER == true
  if (!samplingTimer.attachInterruptInterval(SAMPLE_INTERVAL_uS, samplingTimerHandler)) {
    digitalWrite(LEDR, LOW); // Turn on red LED
    digitalWrite(LEDG, LOW); // Turn on green LED
    while (1);
  }
  samplingTimer.stopTimer();
#endif
  if (!IMU.begin()) {
    digitalWrite(LEDR, LOW); // Turn on red LED
    digitalWrite(LEDG, LOW); // Turn on green LED
    while (1);
  }
  BLE.setLocalName("PsyLink");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(sensorCharacteristic);
  sensorService.addCharacteristic(channelCountCharacteristic);
  sensorService.addCharacteristic(axesCharacteristic);
  sensorService.addCharacteristic(imuCharacteristic);
  BLE.addService(sensorService);
  BLE.setEventHandler(BLEConnected, bleConnectHandler);
  BLE.setEventHandler(BLEDisconnected, bleDisconnectHandler);
  //sensorCharacteristic.setEventHandler(BLERead, sensorCharacteristicRead);
  sensorCharacteristic.writeValue("0");
  channelCountCharacteristic.writeValue(CHANNELS);
  BLE.setConnectionInterval(BLE_CONNECTION_INTERVAL_MIN, BLE_CONNECTION_INTERVAL_MAX);
  BLE.advertise();
  for (int i = 0; i < BLE_CHARACTERISTIC_SIZE; i++) {
    bleString[i] = i + 1;
  }


  //  Serial.println("Initializing flash storage...");

  //Flash storage initialization
  const uint32_t flash_buffer_address = reinterpret_cast<uint32_t>(flash_buffer);
  static FlashIAPBlockDevice flashBlockDevice(flash_buffer_address, kFlashBufferSize);
  flashBlockDevice.init();
  uint8_t* ram_buffer = (uint8_t*)(malloc(kFlashBufferSize));
  flashBlockDevice.read(ram_buffer, 0, kFlashBufferSize);
  flashBlockDevice.erase(0, kFlashBufferSize);

  biasData = reinterpret_cast<bias_values*>(ram_buffer);

  //  Serial.println(biasData->accelBiasX);

  Wire1.begin();
  if (biasData->calibrated) {
    if (serialFlag) {
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
      writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);
      Serial.println("LSM9DS1 9-axis motion sensor...");
      byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 accel/gyro
      //      Serial.print("LSM9DS1 accel/gyro"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x68, HEX);
      byte d = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 magnetometer
      //      Serial.print("LSM9DS1 magnetometer"); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x3D, HEX);
      // Read the WHO_AM_I registers, this is a good test of communication
      if (c == 0x68 && d == 0x3D) { // WHO_AM_I should always be 0x0E for the accel/gyro and 0x3C for the mag
        Serial.println("LSM9DS1 is online...");
        getAres();
        getGres();
        getMres();
        Serial.println("Perform gyro and accel self test");
        selftestLSM9DS1(); // check function of gyro and accelerometer via self test
        initLSM9DS1(); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        Serial.println("accel biases (mg x 1000)");
        Serial.println(1000.*biasData->accelBiasX);
        Serial.println(1000.*biasData->accelBiasY);
        Serial.println(1000.*biasData->accelBiasZ);
        Serial.println("gyro biases (dps)");
        Serial.println(biasData->gyroBiasX);
        Serial.println(biasData->gyroBiasY);
        Serial.println(biasData->gyroBiasZ);
        Serial.println("mag biases (mG x 1000)");
        Serial.println(1000.*biasData->magBiasX);
        Serial.println(1000.*biasData->magBiasY);
        Serial.println(1000.*biasData->magBiasZ);


        Serial.println("LSMDS1 calibrated and ready to go.");
      }
      else {
        Serial.println("LSM9DS1 is offline...");
      }
    }
    else {
      writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
      writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);
      byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 accel/gyro
      byte d = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 magnetometer
      // Read the WHO_AM_I registers, this is a good test of communication
      if (c == 0x68 && d == 0x3D) { // WHO_AM_I should always be 0x0E for the accel/gyro and 0x3C for the mag
        getAres();
        getGres();
        getMres();
        selftestLSM9DS1(); // check function of gyro and accelerometer via self test
        initLSM9DS1(); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      }
      else {
        while (1) {
          delay(100);
          digitalWrite(myLed, HIGH);
          delay(100);
          digitalWrite(myLed, LOW);
        }
      }
    }
  } else {
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);

    delay(100);
    Serial.println("LSM9DS1 9-axis motion sensor...");
    byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 accel/gyro
    Serial.print("LSM9DS1 accel/gyro"); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x68, HEX);
    byte d = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 magnetometer
    Serial.print("LSM9DS1 magnetometer"); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x3D, HEX);

    if (c == 0x68 && d == 0x3D) // WHO_AM_I should always be 0x0E for the accel/gyro and 0x3C for the mag
    {
      // get sensor resolutions, only need to do this once
      getAres();
      getGres();
      getMres();
      Serial.print("accel sensitivity is "); Serial.print(1. / (1000.*aRes)); Serial.println(" LSB/mg");
      Serial.print("gyro sensitivity is "); Serial.print(1. / (1000.*gRes)); Serial.println(" LSB/mdps");
      Serial.print("mag sensitivity is "); Serial.print(1. / (1000.*mRes)); Serial.println(" LSB/mGauss");

      Serial.println("Perform gyro and accel self test");
      selftestLSM9DS1(); // check function of gyro and accelerometer via self test

      Serial.println("Calibrating gyro and accel...");
      accelgyrocalLSM9DS1(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      biasData->gyroBiasX = gyroBias[0];
      biasData->gyroBiasY = gyroBias[1];
      biasData->gyroBiasZ = gyroBias[2];
      biasData->accelBiasX = accelBias[0];
      biasData->accelBiasY = accelBias[1];
      biasData->accelBiasZ = accelBias[2];
      Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
      Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

      magcalLSM9DS1(magBias);
      biasData->magBiasX = magBias[0];
      biasData->magBiasY = magBias[1];
      biasData->magBiasZ = magBias[2];
      Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]);
      delay(2000); // add delay to see results before serial spew of data
      Serial.println("LSM9DS1 calibration complete.");
      initLSM9DS1(); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      Serial.println("LSM9DS1 ready.");
      biasData->calibrated = 1;
      flashBlockDevice.program(ram_buffer, 0, kFlashBufferSize);
    }
    else
    {
      Serial.print("Could not connect to LSM9DS1: 0x");
      Serial.println(c, HEX);
    }
  }

  flashBlockDevice.deinit();
}

unsigned long nextFrame = 0;
void loop() {
#if USE_INTERRUPT_TIMER == true
  if (doSampling) {
#else
  if (micros() >= nextFrame) {
#endif
    readSamples();
#if USE_INTERRUPT_TIMER == false
    nextFrame = micros() + 1000;
#endif
  }
  if (sendBuffer != NO_BUFFER) {
    updateSensorCharacteristic();
  }
  BLE.poll();
}

void sensorCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  updateSensorCharacteristic();
}

#if USE_INTERRUPT_TIMER == true
void samplingTimerHandler() {
  doSampling = true;
}
#endif

void readSamples() {
#if USE_INTERRUPT_TIMER == false
  doSampling = false;
#endif

  for (int channel = 0; channel < CHANNELS; channel++)
    samples[currentBuffer][channel][currentSample] = analogRead(channel);

  currentSample++;

  if (currentSample >= SAMPLES_PER_NOTIFY) {
    sendBuffer = currentBuffer;
    currentBuffer = (currentBuffer + 1) % BUFFERS;
    currentSample = 0;
    tick++;
    if (tick == 0) {
      tick++;
    }
  }

#if SEND_METRICS == true
  unsigned long int currentMicroSeconds = micros();
  unsigned long int sampleDelay = currentMicroSeconds - lastSampleMicroSeconds;
  lastSampleMicroSeconds = currentMicroSeconds;
  if (maxSampleDelay < sampleDelay)
    maxSampleDelay = sampleDelay;
  if (minSampleDelay > sampleDelay)
    minSampleDelay = sampleDelay;
#endif
}
void updateSensorCharacteristic() {
  if (sendBuffer == NO_BUFFER || !bleConnected) return;
  int pos = 0;
  char currentChar;
  float x, y, z;

  // Read configuration pins
  bool enableIMU = digitalRead(JUMPER_PIN_TO_DISABLE_IMU);

  // Metadata
  bleString[pos++] = tick;
#if SEND_METRICS == true
  bleString[pos++] = (COMPRESS_DELAY(minSampleDelay) << 4) | COMPRESS_DELAY(maxSampleDelay);
#else
  bleString[pos++] = 0xFF;
#endif


  ////////////////////////////////

  int quaternionStart = millis();

  readGyroData(gyroCount);  // Read the x/y/z adc values
  // Calculate the gyro value into actual degrees per second
  gx = (float)gyroCount[0] * gRes - biasData->gyroBiasX; // get actual gyro value, this depends on scale being set
  gy = (float)gyroCount[1] * gRes - biasData->gyroBiasY;
  gz = (float)gyroCount[2] * gRes - biasData->gyroBiasZ;

  readAccelData(accelCount);  // Read the x/y/z adc values
  // Now we'll calculate the accleration value into actual g's
  ax = (float)accelCount[0] * aRes - biasData->accelBiasX; // get actual g value, this depends on scale being set
  ay = (float)accelCount[1] * aRes - biasData->accelBiasY;
  az = (float)accelCount[2] * aRes - biasData->accelBiasZ;

  readMagData(magCount);  // Read the x/y/z adc values
  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  mx = (float)magCount[0] * mRes; // - biasData->magBiasX; // TODO maybe include - check to see if bias is in non-volatile register
  my = (float)magCount[1] * mRes; // - biasData->magBiasY;
  mz = (float)magCount[2] * mRes; // - biasData->magBiasZ;

  dataPacketIMU.ax = ax;
  dataPacketIMU.ay = ay;
  dataPacketIMU.az = az;
  dataPacketIMU.gx = gx;
  dataPacketIMU.gy = gy;
  dataPacketIMU.gz = gz;  
  dataPacketIMU.mx = mx;
  dataPacketIMU.my = my;
  dataPacketIMU.mz = mz;

  imuCharacteristic.writeValue((byte *) &dataPacketIMU, sizeof(dataPacketIMU));



  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, -mx, my, mz);

  tempCount = readTempData();  // Read the gyro adc values
  temperature = ((float) tempCount / 256. + 25.0); // Gyro chip temperature in degrees Centigrade

  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= 180.0f / PI;
  yaw   *= 180.0f / PI;
  //  yaw   -= -5.92; // Declination at Tampa, FL
  yaw   -= 15.39; // Declination at Seattle, WA
  roll  *= 180.0f / PI;

  dataPacketAxes.yaw = yaw;
  dataPacketAxes.pitch = pitch;
  dataPacketAxes.roll = roll;
  axesCharacteristic.writeValue((byte *) &dataPacketAxes, sizeof(dataPacketAxes));

//  Serial.print("Yaw, Pitch, Roll: ");
//  Serial.print(yaw, 2);
//  Serial.print(", ");
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.println(roll, 2);

  //  Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz\n"); 30 to 40Hz

  count = millis();
  sumCount = 0;
  sum = 0;

  //  Serial.print("Quaternion calculation time (ms): ");   // 8 - 20 ms
  //  Serial.println(count - quaternionStart);

  //////////////////////////////////////////////


  // Sample data
  for (int sample = 0; sample < SAMPLES_PER_NOTIFY; sample++) {
    for (int channel = 0; channel < CHANNELS; channel++) {
      // Avoid writing 0x00 since that denotes the end of the string
      currentChar = map(samples[sendBuffer][channel][sample], 1040, 3080, 1, 255);
      // Serial.print(samples[sendBuffer][channel][sample]);
      // Serial.print(' ');
      bleString[pos++] = max(1, min(currentChar, 255));
    }
    // Serial.print('\t');
  }
  //  Serial.print("pos: ");
  //  Serial.println(pos);
  bleString[pos] = 0;  // End the string with 0x00
  sensorCharacteristic.writeValue(bleString);
  //  Serial.println(bleString);

  // Reset values
  sendBuffer = NO_BUFFER;
#if SEND_METRICS == true
  minSampleDelay = 999999999;
  maxSampleDelay = 0;
  lastSampleMicroSeconds = micros();
#endif


}

void bleConnectHandler(BLEDevice central) {
  analogWrite(LEDB, 253);
  connectedDevice = central;
  bleConnected = true;
  currentSample = 0;
  currentBuffer = 0;
#if SEND_METRICS == true
  minSampleDelay = 999999999;
  maxSampleDelay = 0;
#endif
  tick = 1;
  sendBuffer = NO_BUFFER;
  for (int buf = 0; buf < BUFFERS; buf++)
    for (int channel = 0; channel < CHANNELS; channel++)
      for (int sample = 0; sample < SAMPLES_PER_NOTIFY; sample++)
        samples[buf][channel][sample] = 0;
  //samplingTimer.restartTimer();
}

void bleDisconnectHandler(BLEDevice central) {
  //samplingTimer.stopTimer();
  analogWrite(LEDB, 255);
  bleConnected = false;
}
