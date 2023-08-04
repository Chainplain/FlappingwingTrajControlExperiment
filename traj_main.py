#"""chainplan multithread traj control,"""
#2023-1-5 15:39:20
# by chainplain

import schedule
import time
import numpy as np
import serial
import threading

from   SpecialorthogonalControl import SO_3_controller as SOC
from   FilterLib import Low_Pass_Second_Order_Filter as LPSF
from   FilterLib import FIR_Filter
from   FTslideControl import Finite_time_slide_mode_observer_3dim as FT_observer
from   FTslideControl import Positional_Traj_Track_Controller as  PTTC
from   FTslideControl import Computing_desired_rotation
from   FTslideControl import Attitude_reference_generator as ARG
from   QualisysConnector import Qualisys_con
from   SerialTRAN import Serial_transmit_multi_protocol as STM
from   RotationComputation import FromEuler_Angle_in_Rad2Rotation as E2R
from   RotationComputation import Angle_Trajectory_Generator as ATG
from   RotationComputation import CalcAngularVelocity_in_Rad_From_RotationMatrixDiff as RD2AV
from   RotationComputation import Normalize_Rot_Mat_Safe
import socket
import scipy.io as scio

hostname = socket.gethostname()
RecordTime = time.strftime('%Y_%m_%d_%H_%M_%S', time.localtime(time.time()))
Record_file_name = hostname + '_' + RecordTime + 'FlapperInQualisysTraj.mat'

Program_life_length = 40 # in seconds

Millimeter2Meter_CONSTANT = 0.001
SELF_HOLD_FLAPPING_FREQ = 14
ANALYZING_SHORTEST_TIME_in_SEC = 0.1

Desired_Controller_gap =  0.02
Desired_Sensor_gap     =  0.01
Desired_Distribute_gap =  0.02
Desired_Analyze_gap    =  0.5
Desired_Record_gap     =  0.02

Pending_gap            =  0.002

Controller_gap =  Desired_Controller_gap
Sensor_gap     =  Desired_Sensor_gap
Distribute_gap =  Desired_Distribute_gap

start_stamp    = time.time()
last_analyzing_stamp = start_stamp


Flapper_Robot_Mass     =  0.029

Angle_vel = np.matrix([[0.0],[0.0],[0.0]])

Identical_rot = np.matrix([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
Zero_av = np.matrix([[0.0],[0.0],[0.0]])

Z_pos_Int = 0


### Classes Initialization
Here_pos_observer = FT_observer(robot_mass = Flapper_Robot_Mass)
Here_pos_observer. observer_gap = Sensor_gap

SO3_Attitude_Controller = SOC()
SO3_Attitude_Controller.time_step = Controller_gap

Here_ARG = ARG()
Here_ARG. generator_gap_AV          = Controller_gap
Here_ARG. generator_gap_rotation    = Sensor_gap


Postion_Controller = PTTC(Flapper_Robot_Mass, Desired_Controller_gap)
Postion_Controller.Control_gap = Controller_gap

Flapper_att_filter = FIR_Filter(Identical_rot, [1] * 20, Desired_Sensor_gap)
Flapper_pos_filter = FIR_Filter(Zero_av, [1] * 10, Desired_Sensor_gap)


Flapper_av_filter = FIR_Filter(Zero_av, [1] * 20, Desired_Sensor_gap)

Angular_velocity_filter = LPSF(Zero_av, 8, 0.8, Desired_Sensor_gap)

TCP_IP = '192.168.1.142'
UDP_IP_port = ('192.168.1.147', 6666)
TCP_PORT = 22221
# threading.main_thread()

here_qualisys_con = Qualisys_con(TCP_IP, TCP_PORT, UDP_IP_port)
time.sleep(1) # Waiting for connection setup

here_qualisys_con. start_capture()
here_qualisys_con. start_listening()
# Because the sensor is asynchronous (running on another machine),
# we have to use multithread reading,
# instead of waiting for data.

leftwing_dorsal_PWM = 900
leftwing_vental_PWM = 2100
leftwing_channel    = 1
### in the real RC channel is 1, always plus 1, these value should be checked by real experiment.

rightwing_dorsal_PWM = 900
rightwing_vental_PWM = 2100
rightwing_channel    = 3

flap_max_PWM = 900
flap_min_PWM = 2100
flap_channel    = 2

rudder_left_PWM = 2100
rudder_right_PWM = 900
rudder_channel    = 0


MultiProtocol_ser = serial.Serial(
        port='COM8',
        baudrate=100000,
        parity= serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )

K_roll = 0.5
K_pitch = 1
K_yaw = 1

#distributing
throttle_com = 0
roll_com   = 0
pitch_com = 0
yaw_com   = 0


# analyzing
is_fall_flag = False
sensoring_count = 0
controlling_count = 0
distributing_count = 0

stop_sensoring = False
stop_controlling = False
stop_distributing = False
stop_recording = False

Sensor_data = [0.0] * 6
Output_channel_data = [1500.0] * 16
Output_channel_data[2] = 1000  # set throttle low

p_d = np.mat([  [0.0], [0.0], [0.0]])
u_t = np.mat([  [0.0], [0.0], [0.0]])
Flapper_pos = np.mat([  [0.0], [0.0], [0.0]])
Flapper_att = np.mat([  [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

Last_pos = np.mat([  [0.0], [0.0], [0.0]])
Last_att = np.mat([  [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

R_d = np.mat([  [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
omega_d = np.matrix([[0],[0],[1]])

Torward_direction = np.mat([  [1], [0], [0]])

record_Sensor_data_list = []
record_com_list = []
record_Output_channel_data_list = []
record_R_d_list = []
record_p_list   = []
record_time_stamp_list = []
record_p_d_list = []
record_Flapper_att_list = []
record_Angle_vel_list = []
record_Observer_p_list = []
record_Observer_v_list = []
record_Observer_z_list = []
record_u_t_list = []


Here_ATG = ATG(R_d, omega_d, Desired_Controller_gap)

def Warm_up_vehicle():
    Warm_up_max_PWM = 1200
    Warm_up_min_PWM = 1800
    # rightwing_channel = 2

    setting = [0, 21, 0, 1, 1, 0, 0, 0]
    op = 6
    Duration = 100
    print('Warming up!!')
    print('Low flap!!')
    for i in range(0, Duration):
        D = [1500] * 16
        D[flap_channel] = Warm_up_min_PWM
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)
        # print('\n')
    print('Gradually Changing.\n')
    for i in range(0, Duration):
        D = [1500] * 16
        D[flap_channel] = Warm_up_min_PWM + i / Duration * (Warm_up_max_PWM - Warm_up_min_PWM)
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)
    print('High flap!!')
    for i in range(0, Duration):
        D = [1500] * 16
        D[flap_channel] = Warm_up_max_PWM
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)
    print('Gradually Changing.\n')
    for i in range(0, Duration):
        D = [1500] * 16
        D[flap_channel] = Warm_up_max_PWM + i / Duration * (Warm_up_min_PWM - Warm_up_max_PWM)
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)

    # MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, Output_channel_data)).hex()))

def Sensoring():
        global Flapper_pos, Flapper_att, Last_pos, Last_att, Sensor_data, throttle_com, sensoring_count, is_fall_flag
        Sensor_data = here_qualisys_con. read_6DEuler_LittleEndian()
        Flapper_pos = Millimeter2Meter_CONSTANT * np.mat([[Sensor_data[0]], [Sensor_data[1]], [Sensor_data[2]]])
        Flapper_pos_filter.march_forward(Flapper_pos)
        Flapper_pos = Flapper_pos_filter.Get_filtered()


        Flapper_att = E2R(np.deg2rad(Sensor_data[3:6]))
        Flapper_att_filter.march_forward(Flapper_att)
        Flapper_att = Flapper_att_filter.Get_filtered()

        # print('Flapper_att_filtered', Flapper_att)
        # print('Flapper_att_filter.Get_filtered:', Flapper_att_filter.Get_filtered())
        Flapper_att = Normalize_Rot_Mat_Safe(Flapper_att)

        if is_fall_flag and \
            0.5 < Flapper_pos[2, 0] < 2 and \
            -2 < Flapper_pos[1, 0] < 2 and \
            -2 < Flapper_pos[0, 0] < 2:
            is_fall_flag = False

        if Flapper_pos[2, 0] < 0.2 or Flapper_pos[2, 0] > 3:
            is_fall_flag = True

        # if here_qualisys_con.loop_delay > 20:
        #     is_fall_flag = True

        if Flapper_pos[1, 0] < -3 or Flapper_pos[1, 0] > 3:
            is_fall_flag = True

        if Flapper_pos[0, 0] < -3 or Flapper_pos[0, 0] > 3:
            is_fall_flag = True







        # if here_qualisys_con.loop_delay > 20:
        #         is_fall_flag = True
        # else:
        #         if here_qualisys_con.loop_delay < 10:
        #                 is_fall_flag = False

        # if Flapper_pos[0,0] > 2 or Flapper_pos[0,0] > -2:
        #         is_fall_flag = True
        # else:
        #         if Flapper_pos[0,0] < 1.8 and Flapper_pos[0,0] > -1.8:
        #                 is_fall_flag = False
        #
        # if Flapper_pos[1, 0] > 2 or Flapper_pos[1, 0] > -2:
        #     is_fall_flag = True
        # else:
        #     if Flapper_pos[1, 0] < 1.8 and Flapper_pos[1, 0] > -1.8:
        #         is_fall_flag = False

        Flapper_Angular_velocity_current = RD2AV(Last_att, Flapper_att, Sensor_gap)

        Flapper_av_filter.march_forward( Flapper_Angular_velocity_current )
        Flapper_Angular_velocity_temp = Flapper_av_filter.Get_filtered()
        Angular_velocity_filter.march_forward(Flapper_Angular_velocity_temp)
        # print('Flapper_Angular_velocity_current', Flapper_Angular_velocity_current)
        # print('Flapper_Angular_velocity_current filtered', Angular_velocity_filter.Get_filtered())
        Last_att    = Flapper_att

        u_t_in_body_fixed_frame = np.mat([[0], [0], [Flapper_Robot_Mass * 9.8]])
        u_t_in_inertia_frame = Flapper_att * u_t_in_body_fixed_frame
        Here_pos_observer.march_forward(u_t_in_inertia_frame, Flapper_pos)
        Here_ARG.match_forward_rotation()

        sensoring_count += 1

def Controlling():
        # Circular Flight
        global throttle_com, roll_com, pitch_com, yaw_com, controlling_count, R_d, p_d, Angle_vel, u_t, Z_pos_Int
        current_stamp = time.time()

        k_rate = 1
        t_rate = 0.2
        p_d_z = 1.5
        v_d_z = 0
        d_v_d_z = 0
        
        Here_time = current_stamp - start_stamp
        
        p_d_x = - k_rate * np.cos(np.pi * t_rate * Here_time) + k_rate
        p_d_y = k_rate * np.sin(np.pi * t_rate * Here_time)
        
        v_d_x = k_rate * np.pi * t_rate * np.sin(np.pi * t_rate * Here_time)
        v_d_y = k_rate * np.pi * t_rate * np.cos(np.pi * t_rate * Here_time)
        
        d_v_d_x = k_rate * np.pi * t_rate * np.pi * t_rate * np.cos(np.pi * t_rate * Here_time)
        d_v_d_y = - k_rate * np.pi * t_rate * np.pi * t_rate * np.sin(np.pi * t_rate * Here_time)
        
        p_d = np.mat([[p_d_x], [p_d_y], [p_d_z]])
        v_d = np.mat([[v_d_x], [v_d_y], [v_d_z]])
        d_v_d = np.mat([[d_v_d_x], [d_v_d_y], [d_v_d_z]])

        # p_d = np.mat([[0], [0], [1.5]])
        # v_d = np.mat([[0], [0], [0]])
        # d_v_d = np.mat([[0], [0], [0]])

        # Flight_direction = np.mat([[v_d_x], [v_d_y], [0]])
        Flight_direction = np.mat([[1], [0], [0]])

        Angle_vel = Angular_velocity_filter.Get_filtered()

        u_t = Postion_Controller.Calc_u_t(p_d, Flapper_pos_filter.Get_filtered(),
                                          v_d, Here_pos_observer.v_observer,
                                          d_v_d, Here_pos_observer.z_observer,
                                          Flapper_att)


        R_d = Computing_desired_rotation(u_t, Torward_direction, Flight_direction)

        Here_ARG.match_forward_angular_velcoity(R_d)

        # print('Here_ARG.Omega_f', Here_ARG.Omega_f)
        # SO3_Attitude_Controller. Generate_control_signal( Flapper_att, Angle_vel,
        #                                  R_d, Here_ARG.Omega_f)
        # R_ident= np.mat([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])

        # Here_ATG. march_forward(Here_ATG. orientation, Here_ATG. omega)
        # R_rot_Y_20_deg = np.mat([[0.9396926, 0.0000000, 0.3420202],
        #                          [0.0000000, 1.0000000, 0.0000000],
        #                          [-0.3420202, 0.0000000, 0.9396926]])
        A_zero = np.mat([[ 0.0], [0.0], [0.0]])
        SO3_Attitude_Controller. Generate_control_signal( Flapper_att, Angle_vel,
                                        R_d, Here_ARG. omega)

        K_p_throttle_com = 3
        K_d_throttle_com = 1.5
        K_i_throttle_com = 0.02
        I_sat = 20

        throttle_com = (p_d[2, 0] - Flapper_pos_filter.Get_filtered()[2, 0]) * K_p_throttle_com\
                       + (v_d[2, 0] - Here_pos_observer.v_observer[2, 0] ) * K_d_throttle_com +\
                       Z_pos_Int * K_i_throttle_com
        Z_pos_Int += (p_d[2, 0] - Flapper_pos_filter.Get_filtered()[2, 0]) * Controller_gap

        if Z_pos_Int > I_sat:
            Z_pos_Int = I_sat
        if Z_pos_Int < -I_sat:
            Z_pos_Int = -I_sat

        if throttle_com > 1:
            throttle_com = 1
        if throttle_com < -0.2:
            throttle_com = -0.2

        # throttle_com = 1 / 9.8 * np.linalg.norm (u_t)
        # roll_com  = K_roll * SO3_Attitude_Controller.u[0,0]
        # pitch_com = K_pitch * SO3_Attitude_Controller.u[1,0]
        # yaw_com   = K_yaw * SO3_Attitude_Controller.u[2,0]

        yaw_com  =  K_yaw * SO3_Attitude_Controller.u[0,0]
        pitch_com =  K_pitch * SO3_Attitude_Controller.u[1,0]
        roll_com   =  K_roll * SO3_Attitude_Controller.u[2,0]

        if yaw_com > 1:
            yaw_com = 1
        if yaw_com < -1:
            yaw_com = -1

        if pitch_com > 1:
            pitch_com = 1
        if pitch_com < -1:
            pitch_com = -1

        if roll_com > 1:
            roll_com = 1
        if roll_com < -1:
            roll_com = -1



        controlling_count += 1

def Distributing():
        # we use a indicator to know
        global throttle_com, roll_com, pitch_com, yaw_com, distributing_count, Output_channel_data
        # Output_channel_data[flap_channel] = flap_min_PWM
        Output_channel_data[flap_channel] = (1500 - 700 * throttle_com)

        Output_channel_data[leftwing_channel] = 1500 - 500 * pitch_com + 300 * yaw_com
        Output_channel_data[rudder_channel] = 1500 - 700 * roll_com
        Output_channel_data[rightwing_channel] = 1500 - 500 * pitch_com - 300 * yaw_com
        # print('roll_com' + str(roll_com) + 'pitch_com' + str(pitch_com) + 'yaw_com' + str(yaw_com) )

        # leftwing_dorsal_PWM = 900
        # leftwing_vental_PWM = 2100
        # leftwing_channel = 0
        ### in the real RC channel is 1, always plus 1, these value should be checked by real experiment.

        # rightwing_dorsal_PWM = 900
        # rightwing_vental_PWM = 2100
        # rightwing_channel = 3
        #
        # flap_max_PWM = 900
        # flap_min_PWM = 2100
        # flap_channel = 2
        #
        # rudder_left_PWM = 2100
        # rudder_right_PWM = 900
        # rudder_channel = 1

        Output_channel_Ex_Max_list = []
        Output_channel_Ex_Min_list = []

        Output_channel_Ex_Max_list.append(leftwing_vental_PWM)
        Output_channel_Ex_Max_list.append(rudder_left_PWM)
        Output_channel_Ex_Max_list.append(flap_min_PWM)
        Output_channel_Ex_Max_list.append(rightwing_vental_PWM)

        Output_channel_Ex_Min_list.append(leftwing_dorsal_PWM)
        Output_channel_Ex_Min_list.append(rudder_right_PWM)
        Output_channel_Ex_Min_list.append(flap_max_PWM)
        Output_channel_Ex_Min_list.append(rightwing_dorsal_PWM)

        for i in range(4):
                Output_channel_data[i] = int(Output_channel_data[i])
                if Output_channel_data[i] < Output_channel_Ex_Min_list[i]:
                    Output_channel_data[i] = Output_channel_Ex_Min_list[i]
                if Output_channel_data[i] > Output_channel_Ex_Max_list[i]:
                    Output_channel_data[i] = Output_channel_Ex_Max_list[i]
        setting = [0, 21, 0, 1, 1, 0, 0, 0]
        op = 6

        if is_fall_flag:
                Output_channel_data[flap_channel] = flap_min_PWM

        # Output_channel_data[flap_channel] = flap_min_PWM

        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, Output_channel_data)).hex()))

        distributing_count += 1

def Analyzing():
        global last_analyzing_stamp, sensoring_count, controlling_count, distributing_count, \
                Controller_gap, Sensor_gap, Distribute_gap, throttle_com, roll_com, pitch_com, yaw_com, Output_channel_data
        here_stamp = time.time()
        elapsed_analyzing_time = here_stamp - last_analyzing_stamp
        last_analyzing_stamp = here_stamp
        if elapsed_analyzing_time < ANALYZING_SHORTEST_TIME_in_SEC:
           return
        # find the real gap, since the system is not RTOS
        Controller_gap = elapsed_analyzing_time / controlling_count
        Sensor_gap     = elapsed_analyzing_time / sensoring_count
        Distribute_gap = elapsed_analyzing_time / distributing_count

        print('INFO:Sensor_gap: '+ str(Sensor_gap) + ', Controller_gap: '+ str(Controller_gap) +\
              ', Distribute_gap: ' + str(Distribute_gap))
        # print('Angular_velocity_filter',Angular_velocity_filter.Get_filtered())

        SO3_Attitude_Controller.time_step = Controller_gap

        Here_ARG.generator_gap_AV = Controller_gap
        Here_ARG.generator_gap_rotation = Sensor_gap

        Postion_Controller.Control_gap = Controller_gap
        Here_pos_observer.observer_gap = Sensor_gap


        # Angular_velocity_filter.filter_gap = Sensor_gap
        #Please be careful to deal with this line.

        controlling_count  = 0
        sensoring_count    = 0
        distributing_count = 0

def Recording():
    global Sensor_data, Output_channel_data, R_d, p_d, Angle_vel, Flapper_pos, throttle_com,\
            roll_com, pitch_com, yaw_com, start_stamp, u_t
        # record_Sensor_data_list, record_Output_channel_data_list, record_R_d_list, record_p_d_list

    record_Sensor_data_list.append(Sensor_data)
    record_Flapper_att_list.append(Flapper_att)
    record_p_list.append(Flapper_pos)

    record_com_list.append([throttle_com, roll_com, pitch_com, yaw_com])
    record_Output_channel_data_list.append(Output_channel_data[0:4])
    record_R_d_list.append(R_d)
    record_p_d_list.append(p_d)
    record_Angle_vel_list.append(Angle_vel)

    record_Observer_p_list. append(Here_pos_observer.p_observer)
    record_Observer_v_list. append(Here_pos_observer.v_observer)
    record_Observer_z_list. append(Here_pos_observer.z_observer)
    record_u_t_list.append(u_t)

    record_time_stamp_list. append(time.time() - start_stamp)


def sensoring_listen_func():
    schedule.every(Desired_Sensor_gap).seconds.do(Sensoring)
    while True:
        if stop_sensoring:
            break
        schedule.run_pending()
        # if you want to check pending as soon as possible,
        # note the following line.
        # However, due to the fact that the checking still consume the machine
        # computing resources, the final performance is not determined.
        time.sleep(Pending_gap)

def controlling_listen_func():
    schedule.every(Desired_Controller_gap).seconds.do(Controlling)
    while True:
        if stop_controlling:
            break
        schedule.run_pending()
        # if you want to check pending as soon as possible,
        # note the following line
        time.sleep(Pending_gap)

def distributing_listen_func():
    schedule.every(Desired_Distribute_gap).seconds.do(Distributing)
    while True:
        if stop_distributing:
            break
        schedule.run_pending()
        # if you want to check pending as soon as possible,
        # note the following line
        time.sleep(Pending_gap)

def recording_listen_func():
    schedule.every(Desired_Record_gap).seconds.do(Recording)
    while True:
        if stop_recording:
            break
        schedule.run_pending()
        # if you want to check pending as soon as possible,
        # note the following line
        time.sleep(Pending_gap)


Warm_up_vehicle()





sensor_thread = threading.Thread(target=sensoring_listen_func, daemon=True)
control_thread = threading.Thread(target=controlling_listen_func, daemon=True)
distribute_thread = threading.Thread(target=distributing_listen_func, daemon=True)
record_thread = threading.Thread(target=recording_listen_func, daemon=True)

sensor_thread.start()
print("STATUS: Wait for sensor initializing.")
for i in range(5):
    print("Count down "+ str(5 - i) + " ...")
    time.sleep(1)


start_stamp    = time.time()
control_thread.start()
distribute_thread.start()
record_thread.start()
schedule.every(Desired_Analyze_gap).seconds.do(Analyzing)

while True:
    schedule.run_pending()
    if (time.time() - start_stamp) > Program_life_length:
        stop_distributing = True
        stop_sensoring = True
        stop_controlling = True
        stop_recording = True
        break
    time.sleep(0.1)


scio.savemat(Record_file_name, {'record_Sensor_data': record_Sensor_data_list,
                                'record_Output_channel_data': record_Output_channel_data_list,
                                'record_com': record_com_list,
                                'record_R_d': record_R_d_list,
                                'record_p_d': record_p_d_list,
                                'record_time_stamp': record_time_stamp_list,
                                'record_Flapper_att': record_Flapper_att_list,
                                'record_Angle_vel': record_Angle_vel_list,
                                'record_p': record_p_list,
                                'record_Observer_p':record_Observer_p_list,
                                'record_Observer_v':record_Observer_v_list,
                                'record_Observer_z':record_Observer_z_list,
                                'record_u_t':record_u_t_list})
