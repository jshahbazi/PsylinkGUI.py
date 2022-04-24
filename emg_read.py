import asyncio
from bleak import BleakClient, BleakError, BleakScanner

import numpy as np
import math
import time

# BLUETOOTH_ADAPTER = 'hci0'
# DEFAULT_BLE_ADDRESS = 'A6:B7:D0:AE:C2:76'

SAMPLE_RATE = 500  # Hz
SIGNAL_BUFFER_SIZE = 2000
ASSUMED_BLE_LATENCY = 0.1  # seconds
RECORD_EVERY_N_SAMPLES = 1
REDRAW_SIGNALS_DELAY = 100  # milliseconds

# AI Constants
SEED_TENSORFLOW = SEED_NUMPY = 1337
DEFAULT_TRAINING_EPOCHS = 25
DEFAULT_CHANNELS = 1
IMU_CHANNELS = 6  # For gyroscope and accelerometer
FEATURE_BUFFER_SIZE = 2**20
FEATURE_WINDOW_SIZE = int(SAMPLE_RATE / 2)  # Enough samples to fit half a second
LABEL_SEPARATOR = ','
BATCH_SIZE = 64
TRAIN_SPLIT = 0.8


# last_sample_count = 2048
# self.actual_channels = self.BLE_decoder.decode_channel_count(self.BLE.read_channels())


SAMPLE_VALUE_OFFSET = -127
# Keep these in sync with arduino code.
DELAY_PARAM_A = -11.3384217
DELAY_PARAM_B = 1.93093431



DEVICE_UUID = 'D3396FFA-2747-0E6B-1664-D20B0DA87011'
SERVICE_UUID = '0a3d3fd8-2f1c-46fd-bf46-eaef2fda91e4'
SENSOR_UUID = '0a3d3fd8-2f1c-46fd-bf46-eaef2fda91e5'
CHANNEL_COUNT_UUID = '0a3d3fd8-2f1c-46fd-bf46-eaef2fda91e6'

EMG_CHANNELS = 8
SIGNAL_COUNT = EMG_CHANNELS + IMU_CHANNELS


class BLEDecoder:
    def __init__(self, sample_value_offset=SAMPLE_VALUE_OFFSET):
        self.channels = None
        self.emg_channels = 0
        self.sample_value_offset = sample_value_offset
        self.last_tick = None        

    def decode_packet(self, bytes_):
        """
        Takes bytes as argument, as received from the BLE characteristic of Myocular

        returns {
            'channels': Number of channels,
            'tick': a number between 1 and 256 that gets incremented on each chraracteristic update
            'sample_count': Number of samples,
            'samples': np.array([...], dtype=np.int),
        }
        """
        if self.channels is None:
            raise Exception("Please read and decode the characteristic for the"
                " channel count before running this method. Alternatively, set"
                " the channel count manually with e.g. `decoder.channels = 1`")

        channels = self.channels
        tick = bytes_[0]
        delays = bytes_[1]
        min_sampling_delay = self._decompress_delay((delays & 0xf0) >> 4)
        max_sampling_delay = self._decompress_delay(delays & 0x0f)

        lost_packets = 0
        is_duplicate = False
        if self.last_tick is not None:
            is_duplicate = tick == self.last_tick

            # Need to consider overflow of tick value. Its range is between 1 and incl. 255
            lost_packets = min(max(0, tick - self.last_tick - 1), tick + 255 - self.last_tick - 1)
            if lost_packets:
                print(f"Lost packets: {lost_packets}")

        gyroscope_accelerometer = list(bytes_[2:2+IMU_CHANNELS])
        assert len(gyroscope_accelerometer) == IMU_CHANNELS    

        sample_values = bytes_[1:]
        if len(sample_values) % self.emg_channels != 0:
            # ensure it's divisible by number of channels:
            sample_values[-(len(sample_values) % self.emg_channels):] = []
        sample_count = math.floor(len(sample_values) / self.emg_channels)
        samples = np.zeros((sample_count, self.channels), dtype=np.int64)

        channel = 0
        sample_id = 0
        for sample_value in sample_values:
            samples[sample_id][channel] = sample_value + self.sample_value_offset
            channel += 1
            if channel >= self.emg_channels:
                channel = 0
                sample_id += 1

        # Filling in gyroscope/accelerometer channels
        for sample_id, channels in enumerate(samples):
            assert len(channels) == self.emg_channels + IMU_CHANNELS
            channels[self.emg_channels:] = gyroscope_accelerometer

        self.last_tick = tick
        return {
            'channels': self.channels,
            'tick': tick,
            'min_sampling_delay': min_sampling_delay,
            'max_sampling_delay': max_sampling_delay,
            'sample_count': sample_count,
            'is_duplicate': is_duplicate,
            'lost_packets': lost_packets,        
            'samples': samples,
        }

    @staticmethod
    def _decompress_delay(delay):
        # this reverses COMPRESS_DELAY in Arduino code
        return math.exp((delay - DELAY_PARAM_A) / DELAY_PARAM_B)

    def decode_channel_count(self, byte):
        self.emg_channels = int.from_bytes(byte, 'little')
        self.channels = self.emg_channels + IMU_CHANNELS
        print("Channels = %d" % self.channels)
        return self.channels
      






async def run(ble_address):
    fps = 0
    bps = 0
    nextfps = time.time() + 1
    decoder = BLEDecoder()
    device = await BleakScanner.find_device_by_address(ble_address, timeout=20.0)
    if not device:
        raise BleakError(f"A device with address {ble_address} could not be found.")
    async with BleakClient(device) as client:
        read = await client.read_gatt_char(CHANNEL_COUNT_UUID)
        decoder.decode_channel_count(read)

        while True:
            read = await client.read_gatt_char(SENSOR_UUID)
            decoded = decoder.decode_packet(read)
            print(decoded)
            fps += 1
            bps += len(read)
            if time.time() >= nextfps:
                print(f"FPS: {fps}, BPS: {bps}")
                nextfps += 1
                fps = 0
                bps = 0
                

async def main():
    await run(DEVICE_UUID)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Keyboard Interrupt.  Exiting...")
    pass
