import asyncio
from bleak import BleakClient, BleakScanner
import math, time, yaml
import dearpygui.dearpygui as dpg

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
IMU_CHANNELS = 9  # For gyroscope and accelerometer
FEATURE_BUFFER_SIZE = 2**20
FEATURE_WINDOW_SIZE = int(SAMPLE_RATE / 2)  # Enough samples to fit half a second
LABEL_SEPARATOR = ','
BATCH_SIZE = 64
TRAIN_SPLIT = 0.8

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


class EMGGUI():
    def __init__(self, device_config):  
        self.data_queue = asyncio.Queue()
        self.running = False #multiprocessing.Value(c_bool, False)
        self.shutdown_event = asyncio.Event()
        self.is_paused = False
        self.window_size = 1000                                      
        self.signal_time = []
        self.sample_rollover_count = 0
        self.start_time = time.time()
        self.t = 0
        self.device_config = device_config
      
        self.device_name = self.device_config["device"][0]["name"]
        self.device_uuid = self.device_config["device"][0]['properties'][0]['device_uuid']
        self.service_uuid = self.device_config["device"][0]['properties'][1]['service_uuid']
        self.sensor_uuid = self.device_config["device"][0]['properties'][2]['sensor_uuid']
        self.channel_count_uuid = self.device_config["device"][0]['properties'][3]['channel_count_uuid']
        self.emg_channels = self.device_config["device"][0]['properties'][4]['emg_channels']
        self.imu_channels = self.device_config["device"][0]['properties'][5]['imu_channels']
        self.mac_address = self.device_config["device"][0]['properties'][6]['mac_address']
        
        self.emg_x_axis = []
        self.emg_y_axis = []
        self.imu_time_axis = [0.0] * self.window_size
        self.imu_gyro_x = [0.0] * self.window_size
        self.imu_gyro_y = [0.0] * self.window_size
        self.imu_gyro_z = [0.0] * self.window_size
        self.imu_accel_x = [0.0] * self.window_size
        self.imu_accel_y = [0.0] * self.window_size
        self.imu_accel_z = [0.0] * self.window_size   
        self.imu_mag_x = [0.0] * self.window_size
        self.imu_mag_y = [0.0] * self.window_size
        self.imu_mag_z = [0.0] * self.window_size             
        for i in range(self.emg_channels):
            self.emg_x_axis.append([0.0] * self.window_size)
            self.emg_y_axis.append([0.0] * self.window_size)
        self.magnetometer_available = False

        dpg.create_context()        

    def build_gui(self):
        with dpg.font_registry():
            font_regular_12 = dpg.add_font("fonts/Inter-Regular.ttf", 14)
            font_regular_14 = dpg.add_font("fonts/Inter-Regular.ttf", 18)
            font_regular_24 = dpg.add_font("fonts/DroidSansMono.otf", 36)         

        with dpg.theme() as data_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (26, 30, 32), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (36, 40, 42), category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 8, category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8, category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_ChildBorderSize, 0, category=dpg.mvThemeCat_Core)

        with dpg.theme() as stop_button_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_Text, (249, 122, 94), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_Border, (249, 122, 94), category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize, 1, category=dpg.mvThemeCat_Core)

        with dpg.theme() as start_button_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 128, 0), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_Border, (0, 128, 0), category=dpg.mvThemeCat_Core)
                dpg.add_theme_style(dpg.mvStyleVar_FrameBorderSize, 1, category=dpg.mvThemeCat_Core)

        with dpg.theme() as input_theme:
            with dpg.theme_component(dpg.mvAll, enabled_state=True):
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 7, 9, category=dpg.mvThemeCat_Core)
            with dpg.theme_component(dpg.mvAll, enabled_state=False):
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 7, 9, category=dpg.mvThemeCat_Core)

        with dpg.theme() as center_button_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_ButtonTextAlign, 0.5, category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_Button, (36, 40, 42), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (36, 40, 42), category=dpg.mvThemeCat_Core)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (36, 40, 42), category=dpg.mvThemeCat_Core)

        with dpg.window(tag="main_window", width=1440, height=1024) as window:
            dpg.add_text("Psylink EMG", pos=[40, 40])
            dpg.bind_item_font(dpg.last_item(), font_regular_24)

            dpg.add_text("Device UUID", pos=[40, 90])
            dpg.bind_item_font(dpg.last_item(), font_regular_12)
            dpg.add_input_text(default_value=DEVICE_UUID, pos=[35, 110], width=340, tag="device_uuid")
            dpg.bind_item_font("device_uuid", font_regular_14)
            dpg.bind_item_theme(dpg.last_item(), input_theme)

            with dpg.child_window(height=100, width=160, pos=[35, 160]):
                dpg.add_text("Total Channels", pos=[10, 10])
                dpg.bind_item_theme(dpg.last_item(), center_button_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_12)
                dpg.add_button(label = (self.emg_channels + self.imu_channels), pos=[100, 12], width=40, tag="total_channels_value", small=True)
                dpg.bind_item_theme(dpg.last_item(), input_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_14)

                dpg.add_text("EMG Channels", pos=[10, 40])
                dpg.bind_item_theme(dpg.last_item(), center_button_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_12)
                dpg.add_button(label = self.emg_channels, pos=[100, 42], width=40, tag="emg_channels_value", small=True)
                dpg.bind_item_theme(dpg.last_item(), input_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_14)

                dpg.add_text("IMU Channels", pos=[10, 70])
                dpg.bind_item_theme(dpg.last_item(), center_button_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_12)
                dpg.add_button(label = self.imu_channels, pos=[100, 72], width=40, tag="imu_channels_value", small=True)
                dpg.bind_item_theme(dpg.last_item(), input_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_14)   

            # dpg.add_text("IMU Signals", pos=[35, 380])
            # with dpg.plot(pos=[35, 400], height=300, width=350):
            #     dpg.add_plot_axis(dpg.mvXAxis, tag="x_imu", no_tick_labels=True)
            #     dpg.set_axis_limits("x_imu", 0, 30)
            #     dpg.add_plot_axis(dpg.mvYAxis, tag="y_imu")
            #     # dpg.set_axis_limits("y_imu", -10,10)
            #     dpg.add_line_series([], [], label="signal", parent="y_imu", tag="imu1")
            #     # dpg.add_bar_series([], [], label="signal", weight=1, parent="y_imu", tag="imu1")


            dpg.add_button(label="Start", width=202, height=40, pos=[35, 300], show=True, tag="start_button",callback=self.start_collecting_data)
            dpg.bind_item_font(dpg.last_item(), font_regular_14)
            dpg.bind_item_theme(dpg.last_item(), start_button_theme)

            dpg.add_button(label="Stop", width=202, height=40, pos=[35, 300], show=False, tag="stop_button",callback=self.stop_collecting_data)
            dpg.bind_item_font(dpg.last_item(), font_regular_14)
            dpg.bind_item_theme(dpg.last_item(), stop_button_theme)

            with dpg.child_window(height=980, width=980, pos=[420, 40]):   #120
                dpg.add_text("EMG Signal 1", pos=[10, 10])
                with dpg.plot(pos=[10, 30], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis1", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis1")
                    dpg.add_line_series([], [], label="signal", parent="y_axis1", tag="signal_series1")
                dpg.add_text("EMG Signal 2", pos=[10, 130])
                with dpg.plot(pos=[10, 150], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis2", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis2")
                    dpg.add_line_series([], [], label="signal", parent="y_axis2", tag="signal_series2")
                dpg.add_text("EMG Signal 3", pos=[10, 250]) 
                with dpg.plot(pos=[10, 270], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis3", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis3")
                    dpg.add_line_series([], [], label="signal", parent="y_axis3", tag="signal_series3")
                dpg.add_text("EMG Signal 4", pos=[10, 370])
                with dpg.plot(pos=[10, 390], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis4", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis4")
                    dpg.add_line_series([], [], label="signal", parent="y_axis4", tag="signal_series4")
                dpg.add_text("EMG Signal 5", pos=[10, 490])
                with dpg.plot(pos=[10, 510], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis5", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis5")
                    dpg.add_line_series([], [], label="signal", parent="y_axis5", tag="signal_series5")
                # dpg.add_text("EMG Signal 6", pos=[10, 610])
                # with dpg.plot(pos=[10, 630], height=100, width=950):
                #     dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis6", no_tick_labels=True)
                #     dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis6")
                #     dpg.add_line_series([], [], label="signal", parent="y_axis6", tag="signal_series6")
                dpg.add_text("IMU Signal - Accelerometer", pos=[10, 610])
                with dpg.plot(pos=[10, 630], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis6", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis6")
                    dpg.add_line_series([], [], label="signal", parent="y_axis6", tag="imu_ax")
                    dpg.add_line_series([], [], label="signal", parent="y_axis6", tag="imu_ay")
                    dpg.add_line_series([], [], label="signal", parent="y_axis6", tag="imu_az")
                # dpg.add_text("EMG Signal 7", pos=[10, 730])
                # with dpg.plot(pos=[10, 750], height=100, width=950):
                #     dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis7", no_tick_labels=True)
                #     dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis7")
                #     dpg.add_line_series([], [], label="signal", parent="y_axis7", tag="signal_series7")
                dpg.add_text("IMU Signal - Gyroscope", pos=[10, 730])                    
                with dpg.plot(pos=[10, 750], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis7", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis7")
                    dpg.add_line_series([], [], label="gx", parent="y_axis7", tag="imu_gx")
                    dpg.add_line_series([], [], label="gy", parent="y_axis7", tag="imu_gy")
                    dpg.add_line_series([], [], label="gz", parent="y_axis7", tag="imu_gz")
                # dpg.add_text("EMG Signal 8", pos=[10, 850])
                # with dpg.plot(pos=[10, 870], height=100, width=950):
                #     dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis8", no_tick_labels=True)
                #     dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis8")
                #     dpg.add_line_series([], [], label="signal", parent="y_axis8", tag="signal_series8")
                dpg.add_text("IMU Signal - Magnetometer", pos=[10, 850])
                with dpg.plot(pos=[10, 870], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis8", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis8")
                    dpg.add_line_series([], [], label="signal", parent="y_axis8", tag="imu_mx")
                    dpg.add_line_series([], [], label="signal", parent="y_axis8", tag="imu_my")
                    dpg.add_line_series([], [], label="signal", parent="y_axis8", tag="imu_mz")
                                        

        dpg.create_viewport(title='EMG', width=1440, height=1064, x_pos=40, y_pos=40)
        dpg.bind_item_theme(window, data_theme)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_exit_callback(self.teardown)
        dpg.set_primary_window("main_window", True)

    async def run(self):
        asyncio.create_task(self.collect_emg_data(DEVICE_UUID))           
        while dpg.is_dearpygui_running():
            await asyncio.sleep(0.001)
            await self.update_plots()
            dpg.render_dearpygui_frame()
        await asyncio.sleep(0.01)
        self.running = False
        self.shutdown_event.set() 
        time.sleep(0.1)      
        dpg.destroy_context()


    def start_collecting_data(self):
        self.running = True
        self.start_time = time.time()        
        dpg.configure_item("start_button", show=False)
        dpg.configure_item("stop_button", show=True)


    def stop_collecting_data(self):
        self.running = False
        self.x_axis = []
        self.y_axis = []        
        dpg.configure_item("start_button", show=True)
        dpg.configure_item("stop_button", show=False)

# the raw EMG data is an integer between 0 and 4095, and I map the range 1040-3080 to 1-255
# so it's downscaled, and 127 is something like a zero-value
# I cut it off just because the signals typically don't reach those values, and I'm trying to maximize the resolution with minimal data loss

# emg signals
# currentChar = map(samples[sendBuffer][channel][sample], 1040, 3080, 1, 255);
# bleString[pos++] = max(1, min(currentChar, 255));
# long map(long x, long in_min, long in_max, long out_min, long out_max) {
#   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
# }

# imu signals
# IMU.readGyroscope(x, y, z);
# bleString[pos++] = min(255, max(1, x+127));
# bleString[pos++] = min(255, max(1, y+127));
# bleString[pos++] = min(255, max(1, z+127))
# IMU.readAcceleration(x, y, z);
# bleString[pos++] = min(255, max(1, 128*x+127));
# bleString[pos++] = min(255, max(1, 128*y+127));
# bleString[pos++] = min(255, max(1, 128*z+127));;

    async def update_plots(self):
        current_queue_size = self.data_queue.qsize()
        if self.running == True and current_queue_size > 0:
            incoming_data = await self.data_queue.get()
            # channels = incoming_data['channels']
            sample_count = incoming_data['sample_count']

            self.t = incoming_data['time'] - self.start_time 

            for current_sample in range(sample_count):              
                samples = incoming_data['samples'][current_sample]
                emg_samples = samples[0:8]
                imu_samples = samples[8:]                
                # if len(samples) == 9:
                #     self.magnetometer_available == True


                for index,value in enumerate(emg_samples):
                    emg_data_point = float(value)
                    if index >= 0 and index <= (self.emg_channels - 4):
                        self.emg_x_axis[index].append(self.t)
                        self.emg_x_axis[index] = self.emg_x_axis[index][-self.window_size:]                
                        self.emg_y_axis[index].append(emg_data_point)
                        self.emg_y_axis[index] = self.emg_y_axis[index][-self.window_size:]   
                        plot_tag = 'signal_series' + str(index + 1)
                        dpg.set_value(plot_tag, [self.emg_x_axis[index], self.emg_y_axis[index]])
                        x_axis_tag = 'x_axis' + str(index + 1)
                        dpg.fit_axis_data(x_axis_tag)
                        y_axis_tag = 'y_axis' + str(index + 1)
                        dpg.set_axis_limits(y_axis_tag, -200, 200)    

                self.imu_time_axis.append(self.t)
                self.imu_time_axis = self.imu_time_axis[-self.window_size:]  

                # accelerometer
                accel_data = [ (x - 127)/128 for x in imu_samples[3:6]]
                accel_x = accel_data[0]
                accel_y = accel_data[1]
                accel_z = accel_data[2]
                self.imu_accel_x.append(accel_x)
                self.imu_accel_x = self.imu_accel_x[-self.window_size:] 
                self.imu_accel_y.append(accel_y)
                self.imu_accel_y = self.imu_accel_y[-self.window_size:] 
                self.imu_accel_z.append(accel_z)
                self.imu_accel_z = self.imu_accel_z[-self.window_size:]                                 

                dpg.set_value("imu_ax", [self.imu_time_axis, self.imu_accel_x])
                dpg.set_value("imu_ay", [self.imu_time_axis, self.imu_accel_y])
                dpg.set_value("imu_az", [self.imu_time_axis, self.imu_accel_z])
                dpg.fit_axis_data("x_axis6")
                dpg.set_axis_limits("y_axis6", -1,1.2)   
                
                # gyroscope
                gyro_data = [ (x - 127) for x in imu_samples[0:3]]
                gyro_x = gyro_data[0]
                gyro_y = gyro_data[1]
                gyro_z = gyro_data[2]
                self.imu_gyro_x.append(gyro_x)
                self.imu_gyro_x = self.imu_gyro_x[-self.window_size:] 
                self.imu_gyro_y.append(gyro_y)
                self.imu_gyro_y = self.imu_gyro_y[-self.window_size:] 
                self.imu_gyro_z.append(gyro_z)
                self.imu_gyro_z = self.imu_gyro_z[-self.window_size:]   

                dpg.set_value("imu_gx", [self.imu_time_axis, self.imu_gyro_x])
                dpg.set_value("imu_gy", [self.imu_time_axis, self.imu_gyro_y])
                dpg.set_value("imu_gz", [self.imu_time_axis, self.imu_gyro_z])
                dpg.fit_axis_data("x_axis7")
                dpg.set_axis_limits("y_axis7", -50,50)                
                # dpg.set_axis_limits_auto("y_axis7")

                if self.magnetometer_available:
                    mag_data = [ (x - 127) for x in imu_samples[6:9]]
                    mag_x = mag_data[0]
                    mag_y = mag_data[1]
                    mag_z = mag_data[2]
                    self.imu_mag_x.append(mag_x)
                    self.imu_mag_x = self.imu_mag_x[-self.window_size:] 
                    self.imu_mag_y.append(mag_y)
                    self.imu_mag_y = self.imu_mag_y[-self.window_size:] 
                    self.imu_mag_z.append(mag_z)
                    self.imu_mag_z = self.imu_mag_z[-self.window_size:]
                    dpg.set_value("imu_mx", [self.imu_time_axis, self.imu_mag_x])
                    dpg.set_value("imu_my", [self.imu_time_axis, self.imu_mag_y])
                    dpg.set_value("imu_mz", [self.imu_time_axis, self.imu_mag_z])
                    dpg.fit_axis_data("x_axis8")
                    dpg.set_axis_limits("y_axis8", -100,100)                       
            
        
    async def collect_emg_data(self, ble_address):
        fps = 0
        bps = 0
        nextfps = time.time() + 1
        decoder = BLEDecoder()
        device = await BleakScanner.find_device_by_address(ble_address, timeout=20.0)
        if not device:
            # raise BleakError(f"A device with address {ble_address} could not be found.")
            print(f"A device with address {ble_address} could not be found.")
            self.teardown()
            return

        try:
            async with BleakClient(device) as client:
                read = await client.read_gatt_char(CHANNEL_COUNT_UUID)
                channel_count = decoder.decode_channel_count(read)
                if channel_count == 17:
                    self.magnetometer_available = True
                    self.imu_channels = 9
                    dpg.configure_item("imu_channels_value", label=int(self.imu_channels))
                dpg.configure_item("total_channels_value", label=int(channel_count))

                while not self.shutdown_event.is_set():
                    read = await client.read_gatt_char(SENSOR_UUID)
                    decoded = decoder.decode_packet(read)
                    if decoded['is_duplicate']:
                        continue # we don't want duplicate data
                    await self.data_queue.put(decoded)
                    # print(decoded)
                    fps += 1
                    bps += len(read)
                    if time.time() >= nextfps:
                        print(f"FPS: {fps}, BPS: {bps}")
                        nextfps += 1
                        fps = 0
                        bps = 0
        except:
            # TODO: handle better
            pass
                

    def teardown(self):
        try:
            self.running = False
            self.shutdown_event.set()
            time.sleep(0.1)
            for task in asyncio.all_tasks():
                task.cancel()
            loop = asyncio.get_event_loop()
            loop.stop()            
        except:
            pass    



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
        samples = []
        for i in range(0,sample_count):
            samples.append([])
            for j in range(0,self.channels):
                samples[i].append(0)

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
            'time': time.time()
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


async def main():
    with open("device_config.yaml", "r") as stream:
        try:
            device_config = yaml.safe_load(stream)
        except Exception as e:
            print(f"Error reading config file: {e}")
            return
    emg = EMGGUI(device_config)
    emg.build_gui()
    await emg.run()


if __name__ == '__main__':
    asyncio.run(main())