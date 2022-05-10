import asyncio, math, time, yaml, struct
from bleak import BleakClient, BleakScanner
import dearpygui.dearpygui as dpg

import numbers, warnings
import numpy as np
from numpy.linalg import norm

SAMPLE_VALUE_OFFSET = -127
# Keep these in sync with arduino code.
DELAY_PARAM_A = -11.3384217
DELAY_PARAM_B = 1.93093431


GyroMeasError = math.pi * (40.0 / 180.0)   # gyroscope measurement error in rads/s (start at 40 deg/s)
GyroMeasDrift = math.pi * (0.0 / 180.0)   # gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
beta = math.sqrt(3.0 / 4.0) * GyroMeasError   # compute beta
zeta = math.sqrt(3.0 / 4.0) * GyroMeasDrift   # compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


size = 5
demo_verticies = [
        [-size, -size, -size],  # 0 near side
        [ size, -size, -size],  # 1
        [-size,  size, -size],  # 2
        [ size,  size, -size],  # 3
        [-size, -size,  size],  # 4 far side
        [ size, -size,  size],  # 5
        [-size,  size,  size],  # 6
        [ size,  size,  size],  # 7
        [-size, -size, -size],  # 8 left side
        [-size,  size, -size],  # 9
        [-size, -size,  size],  # 10
        [-size,  size,  size],  # 11
        [ size, -size, -size],  # 12 right side
        [ size,  size, -size],  # 13
        [ size, -size,  size],  # 14
        [ size,  size,  size],  # 15
        [-size, -size, -size],  # 16 bottom side
        [ size, -size, -size],  # 17
        [-size, -size,  size],  # 18
        [ size, -size,  size],  # 19
        [-size,  size, -size],  # 20 top side
        [ size,  size, -size],  # 21
        [-size,  size,  size],  # 22
        [ size,  size,  size],  # 23
    ]

demo_colors = [
        [255,   0,   0, 150],
        [255, 255,   0, 150],
        [255, 255, 255, 150],
        [255,   0, 255, 150],
        [  0, 255,   0, 150],
        [  0, 255, 255, 150],
        [  0,   0, 255, 150],
        [  0, 125,   0, 150],
        [128,   0,   0, 150],
        [128,  70,   0, 150],
        [128, 255, 255, 150],
        [128,   0, 128, 150]
    ]
    


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
        self.emg_channel_count_uuid = self.device_config["device"][0]['properties'][3]['emg_channel_count_uuid']
        self.axes_uuid = self.device_config["device"][0]['properties'][4]['axes_uuid']
        self.imu_uuid = self.device_config["device"][0]['properties'][5]['imu_uuid']        
        self.emg_channels = self.device_config["device"][0]['properties'][6]['emg_channels']
        self.mac_address = self.device_config["device"][0]['properties'][7]['mac_address']
        
        self.decoder = None

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
        for _ in range(self.emg_channels):
            self.emg_x_axis.append([0.0] * self.window_size)
            self.emg_y_axis.append([0.0] * self.window_size)
        self.magnetometer_available = False

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.gyr_x = 0.0
        self.gyr_y = 0.0
        self.gyr_z = 0.0
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0

        self.q = [1.0, 0.0, 0.0, 0.0]

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
            dpg.add_input_text(default_value=self.device_uuid, pos=[35, 110], width=340, tag="device_uuid")
            dpg.bind_item_font("device_uuid", font_regular_14)
            dpg.bind_item_theme(dpg.last_item(), input_theme)

            with dpg.child_window(height=40, width=160, pos=[35, 160]):
                dpg.add_text("EMG Channels", pos=[10, 10])
                dpg.bind_item_theme(dpg.last_item(), center_button_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_12)
                dpg.add_button(label = self.emg_channels, pos=[100, 12], width=40, tag="emg_channels_value", small=True)
                dpg.bind_item_theme(dpg.last_item(), input_theme)
                dpg.bind_item_font(dpg.last_item(), font_regular_14)

            dpg.add_text("IMU Signal - Accelerometer", pos=[35, 380])
            with dpg.plot(pos=[35, 400], height=100, width=350):
                dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis_acc", no_tick_labels=True)
                dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis_acc")
                dpg.add_line_series([], [], label="signal", parent="y_axis_acc", tag="imu_ax")
                dpg.add_line_series([], [], label="signal", parent="y_axis_acc", tag="imu_ay")
                dpg.add_line_series([], [], label="signal", parent="y_axis_acc", tag="imu_az")

            dpg.add_text("IMU Signal - Gyroscope", pos=[35, 510])                    
            with dpg.plot(pos=[35, 530], height=100, width=350):
                dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis_gyr", no_tick_labels=True)
                dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis_gyr")
                dpg.add_line_series([], [], label="gx", parent="y_axis_gyr", tag="imu_gx")
                dpg.add_line_series([], [], label="gy", parent="y_axis_gyr", tag="imu_gy")
                dpg.add_line_series([], [], label="gz", parent="y_axis_gyr", tag="imu_gz")

            dpg.add_text("IMU Signal - Magnetometer", pos=[35, 640])
            with dpg.plot(pos=[35, 660], height=100, width=350):
                dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis_mag", no_tick_labels=True)
                dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis_mag")
                dpg.add_line_series([], [], label="signal", parent="y_axis_mag", tag="imu_mx")
                dpg.add_line_series([], [], label="signal", parent="y_axis_mag", tag="imu_my")
                dpg.add_line_series([], [], label="signal", parent="y_axis_mag", tag="imu_mz")                
            # dpg.add_text("IMU Signals", pos=[35, 380])
            # with dpg.plot(pos=[35, 400], height=300, width=350):
            #     dpg.add_plot_axis(dpg.mvXAxis, tag="x_imu", no_tick_labels=True)
            #     dpg.set_axis_limits("x_imu", 0, 30)
            #     dpg.add_plot_axis(dpg.mvYAxis, tag="y_imu")
            #     dpg.set_axis_limits("y_imu", -50,50)
            #     # dpg.add_line_series([], [], label="signal", parent="y_imu", tag="imu1")
            #     dpg.add_bar_series([], [], label="signal", weight=1, parent="y_imu", tag="imu1")

            dpg.add_text("Power Module Orientation", pos=[35, 800])
            with dpg.group(pos=[10, 805]):            
                with dpg.drawlist(width=200, height=200, tag="_demo_advanced_drawing_3d"):
                    # with dpg.draw_node(tag="_demo_3dNode1", user_data=[0, 0, 0]):
                    #     dpg.draw_arrow((50, 50), (20, 65), color=(0, 200, 255), thickness=5, size=10)    
                    #     dpg.draw_arrow("drawing1", [velocity_x, velocity_y], [0, 0],  [255, 255, 255], 1, 15, tag="arrow")
                
                    with dpg.draw_layer(depth_clipping=False, cull_mode=dpg.mvCullMode_Back, perspective_divide=True):
                        dpg.set_clip_space(dpg.last_item(), 0, 0, 200, 200, -1.0, 1.0)
                        with dpg.draw_node(tag="_demo_3dNode1", user_data=[0, 0, 0]):
                            dpg.draw_triangle(demo_verticies[1],  demo_verticies[2],  demo_verticies[0], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[1],  demo_verticies[3],  demo_verticies[2], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[7],  demo_verticies[5],  demo_verticies[4], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[6],  demo_verticies[7],  demo_verticies[4], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[9],  demo_verticies[10], demo_verticies[8], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[9],  demo_verticies[11], demo_verticies[10], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[15], demo_verticies[13], demo_verticies[12], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[14], demo_verticies[15], demo_verticies[12], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[18], demo_verticies[17], demo_verticies[16], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[19], demo_verticies[17], demo_verticies[18], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[21], demo_verticies[23], demo_verticies[20], color=[0,0,0.0], fill=demo_colors[2])
                            dpg.draw_triangle(demo_verticies[23], demo_verticies[22], demo_verticies[20], color=[0,0,0.0], fill=demo_colors[2])

                    def _demo_live_3D_drawing():
                        view_matrix = dpg.create_fps_matrix([0, 0, 50], 0.0, 0.0)
                        proj_matrix = dpg.create_perspective_matrix(math.pi*45.0/180.0, 500/500, 0.1, 100)
                        cube_rot = dpg.get_item_user_data("_demo_3dNode1")
                        cube_rot[0] = -self.pitch #cube_rot[0] + 1.0
                        cube_rot[1] = -self.yaw #cube_rot[1] + 1.0
                        cube_rot[2] = -self.roll #cube_rot[2] + 1.0
                        model_matrix = dpg.create_rotation_matrix(math.pi*cube_rot[0]/180.0 , [1, 0, 0])*\
                                                dpg.create_rotation_matrix(math.pi*cube_rot[1]/180.0 , [0, 1, 0])*\
                                                dpg.create_rotation_matrix(math.pi*cube_rot[2]/180.0 , [0, 0, 1])
                        dpg.apply_transform("_demo_3dNode1", proj_matrix*view_matrix*model_matrix)
                        dpg.set_item_user_data("_demo_3dNode1", cube_rot)

                    with dpg.item_handler_registry(tag="__demo_item_reg7"):
                        dpg.add_item_visible_handler(callback=_demo_live_3D_drawing)
                    dpg.bind_item_handler_registry("_demo_advanced_drawing_3d", dpg.last_container())   


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
                dpg.add_text("EMG Signal 6", pos=[10, 610])
                with dpg.plot(pos=[10, 630], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis6", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis6")
                    dpg.add_line_series([], [], label="signal", parent="y_axis6", tag="signal_series6")
                dpg.add_text("EMG Signal 7", pos=[10, 730])
                with dpg.plot(pos=[10, 750], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis7", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis7")
                    dpg.add_line_series([], [], label="signal", parent="y_axis7", tag="signal_series7")
                dpg.add_text("EMG Signal 8", pos=[10, 850])
                with dpg.plot(pos=[10, 870], height=100, width=950):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="x_axis8", no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="y_axis8")
                    dpg.add_line_series([], [], label="signal", parent="y_axis8", tag="signal_series8")

                       

        dpg.create_viewport(title='EMG', width=1440, height=1064, x_pos=40, y_pos=40)
        dpg.bind_item_theme(window, data_theme)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_exit_callback(self.teardown)
        dpg.set_primary_window("main_window", True)

    async def run(self):
        asyncio.create_task(self.collect_emg_data(self.device_uuid))           
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
        dpg.configure_item("start_button", show=True)
        dpg.configure_item("stop_button", show=False)


    async def update_plots(self):
        current_queue_size = self.data_queue.qsize()
        if self.running == True and current_queue_size > 0:
            incoming_data = await self.data_queue.get()
            sample_count = incoming_data['sample_count']
            self.t = incoming_data['time'] - self.start_time 

            for current_sample in range(sample_count):              
                samples = incoming_data['samples'][current_sample]
                emg_samples = samples[0:8]          

                for index,value in enumerate(emg_samples):
                    emg_data_point = float(value)
                    if index >= 0 and index <= (self.emg_channels - 1):
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

                self.imu_gyro_x.append(self.gyr_x)
                self.imu_gyro_x = self.imu_gyro_x[-self.window_size:] 
                self.imu_gyro_y.append(self.gyr_y)
                self.imu_gyro_y = self.imu_gyro_y[-self.window_size:] 
                self.imu_gyro_z.append(self.gyr_z)
                self.imu_gyro_z = self.imu_gyro_z[-self.window_size:]   

                dpg.set_value("imu_gx", [self.imu_time_axis, self.imu_gyro_x])
                dpg.set_value("imu_gy", [self.imu_time_axis, self.imu_gyro_y])
                dpg.set_value("imu_gz", [self.imu_time_axis, self.imu_gyro_z])
                dpg.fit_axis_data("x_axis_gyr")
                dpg.set_axis_limits("y_axis_gyr", -50,50)                

                self.imu_accel_x.append(self.acc_x)
                self.imu_accel_x = self.imu_accel_x[-self.window_size:] 
                self.imu_accel_y.append(self.acc_y)
                self.imu_accel_y = self.imu_accel_y[-self.window_size:] 
                self.imu_accel_z.append(self.acc_z)
                self.imu_accel_z = self.imu_accel_z[-self.window_size:]                                 

                dpg.set_value("imu_ax", [self.imu_time_axis, self.imu_accel_x])
                dpg.set_value("imu_ay", [self.imu_time_axis, self.imu_accel_y])
                dpg.set_value("imu_az", [self.imu_time_axis, self.imu_accel_z])
                dpg.fit_axis_data("x_axis_acc")
                dpg.set_axis_limits("y_axis_acc", -1,1.2)

                self.imu_mag_x.append(self.mag_x)
                self.imu_mag_x = self.imu_mag_x[-self.window_size:] 
                self.imu_mag_y.append(self.mag_y)
                self.imu_mag_y = self.imu_mag_y[-self.window_size:] 
                self.imu_mag_z.append(self.mag_z)
                self.imu_mag_z = self.imu_mag_z[-self.window_size:]
                dpg.set_value("imu_mx", [self.imu_time_axis, self.imu_mag_x])
                dpg.set_value("imu_my", [self.imu_time_axis, self.imu_mag_y])
                dpg.set_value("imu_mz", [self.imu_time_axis, self.imu_mag_z])
                dpg.fit_axis_data("x_axis_mag")
                # dpg.fit_axis_data("y_axis_mag")
                dpg.set_axis_limits("y_axis_mag", -0.2,0.6)


#define AA 0.97         // complementary filter constant
#define A_GAIN 0.0573    // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846   
# 
# complimentary filter
# float lerp(float a, float b, float mu)
# {
#     return a*(1.f - mu) + b*mu;
# }
# float mix_ratio = 0.9f; //90% HF, 10% LF - this needs to be tuned
# float filtered_value = lerp(lf_high_noise_value, hf_low_noise_value, mix_ratio);                 
    # def notification_handler(self, sender, data):
    #     """Simple notification handler which prints the data received."""
    #     converted_data = self.decoder.decode_float_characteristic(data)
    #     print("{0}: {1}".format(sender, converted_data))  

    def axes_notification_handler(self, sender, data):
        """Simple notification handler which prints the data received."""
        axes_data = struct.unpack('<fff', data)
        self.yaw = axes_data[0]
        self.pitch = axes_data[1]
        self.roll = axes_data[2]
        # print(f"yaw: {self.yaw} pitch: {self.pitch} roll: {self.roll}")  

    def imu_notification_handler(self, sender, data):
        """Simple notification handler which prints the data received."""
        imu_data = struct.unpack('<fffffffff', data)
        self.acc_x = imu_data[0]
        self.acc_y = imu_data[1]
        self.acc_z = imu_data[2]
        self.gyr_x = imu_data[3]
        self.gyr_y = imu_data[4]
        self.gyr_z = imu_data[5]
        self.mag_x = imu_data[6]
        self.mag_y = imu_data[7]
        self.mag_z = imu_data[8]                
        # print(f"acc_x: {self.acc_x} acc_y: {self.acc_y} acc_z: {self.acc_z}") 
        # print(f"gyr_x: {self.gyr_x} gyr_y: {self.gyr_y} gyr_z: {self.gyr_z}") 
        # print(f"mag_x: {self.mag_x} mag_y: {self.mag_y} mag_z: {self.mag_z}")  
        
    async def collect_emg_data(self, ble_address):
        # fps = 0
        # bps = 0
        nextfps = time.time() + 1
        self.decoder = BLEDecoder()
        device = await BleakScanner.find_device_by_address(ble_address, timeout=20.0)
        if not device:
            print(f"A device with address {ble_address} could not be found.")
            self.teardown()
            return

        async with BleakClient(device) as client:
            read = await client.read_gatt_char(self.emg_channel_count_uuid)
            emg_channel_count = self.decoder.decode_emg_channel_count(read)
            dpg.configure_item("emg_channels_value", label=int(emg_channel_count))

            await client.start_notify(self.axes_uuid, self.axes_notification_handler)
            await client.start_notify(self.imu_uuid, self.imu_notification_handler)

            try:
                while not self.shutdown_event.is_set():
                    read = await client.read_gatt_char(self.sensor_uuid)
                    decoded = self.decoder.decode_packet(read)                 
                    if decoded['is_duplicate']:
                        continue # we don't want duplicate data
                    await self.data_queue.put(decoded)

                    # fps += 1
                    # bps += len(read)
                    # if time.time() >= nextfps:
                    #     print(f"FPS: {fps}, BPS: {bps}")
                    #     nextfps += 1
                    #     fps = 0
                    #     bps = 0
            except Exception as e:
                print(e)
            
            await client.stop_notify(self.axes_uuid)
            await client.stop_notify(self.imu_uuid)
                

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
        self.emg_channels = 8 # default
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

        emg_data = bytes_[2+self.emg_channels:] 
        if len(emg_data) % self.emg_channels != 0:
            # ensure it's divisible by number of channels:
            emg_data[-(len(emg_data) % self.emg_channels):] = []

        sample_count = math.floor(len(emg_data) / self.emg_channels)

        ### Initialize samples array
        samples = []
        for i in range(0,sample_count):
            samples.append([])
            for _ in range(0,(self.emg_channels)):
                samples[i].append(0)
        #########

        channel = 0
        sample_id = 0
        for sample_value in emg_data:
            samples[sample_id][channel] = sample_value + self.sample_value_offset
            channel += 1
            if channel >= self.emg_channels:
                channel = 0
                sample_id += 1

        # print("samples copied")
        # Filling in gyroscope/accelerometer channels
        # for sample_id, channels in enumerate(samples):
        #     assert len(channels) == self.emg_channels
            # channels[self.emg_channels:] = imu_data
        # print("imu copied")            

        self.last_tick = tick
        return {
            'channels': self.emg_channels,
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

    def decode_emg_channel_count(self, byte):
        self.emg_channels = int.from_bytes(byte, 'little')
        print("EMG Channels = %d" % self.emg_channels)
        return self.emg_channels

    def decode_float_characteristic(self, byte):
        return struct.unpack('<f', byte)[0] # unpack returns a tuple of 1 
            

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
