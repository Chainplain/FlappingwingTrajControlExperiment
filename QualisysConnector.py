# chainplain   2022-12-25  Qualisys motion capture connector
# for 3D object position and attitude requirement
import socket
import struct
import time
import threading
import math

class Qualisys_con():
    def __init__(self, basic_TCP_IP, basic_TCP_port, data_UDP_port, Buffer_size = 1024):
        self. TCP_UDP_common_IP = basic_TCP_IP
        self. basic_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self. basic_socket .connect((basic_TCP_IP, basic_TCP_port))
        self. buffer_size = Buffer_size
        self. UDP_port    = data_UDP_port
        # used for data, we only use udp
        self. data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self. read_count = 0
        self. receive_data = b'We have not received any data yet!!'
        self. data_output_list_last = [0.0] * 6
        # self. self_keep_times = 20
        self. loop_delay = 0


    def start_capture(self, freq = 100, data_port = "6666", data_mode = "6DEuler"):
        freq_str = str( freq )
        start_message = "StreamFrames Frequency:" + freq_str + " UDP:"+data_port + " " + data_mode + "\n"
        self.basic_socket. send(start_message.encode())
        print("INFO: Waiting for " + start_message + ".")
        time.sleep(1)
        print("INFO:   " + start_message + "   started!!!")

        self. data_socket .bind(self. UDP_port)
        print("STATUS: UDP socket binded!!!")

        self. stop_listening_thread = False
        self. listening_thread = threading.Thread(target=self.listen_func, daemon=True)
        print("STATUS: Listening thread built!!!")
        # use daemon, we do not need to wait for listening thread


    def listen_func(self):
        while True:
            data, addr = self. data_socket.recvfrom(self. buffer_size)  # buffer size is 1024 bytes
            # print('data:', ''.join(['%02X ' % b for b in data]))
            self.receive_data = data
            # print("DATA: Received message" + str(self.receive_data)  + "from" + str(addr))
            # time.sleep(0.1)
            # self.read_count = self. read_count + 1
            if self. stop_listening_thread:
                break

    def start_listening(self):
        print("STATUS: Start listening!!")
        self. stop_listening_thread = False
        self. listening_thread.start()
        print("STATUS: Listening started!!")

    def stop_listening(self):
        self. stop_listening_thread = True
        self. listening_thread.join()

    def read_6DEuler_LittleEndian(self):
        data_output_list = []
        if len(self.receive_data) == 64:
            # data_component = self. receive_data[40 :]
            for i in range(40, 64, 4):
                data_b = self.receive_data[i: i + 4]
                data_f = struct.unpack('f', data_b)
                # print( type(data_f[0]))
                data_output_list.append(data_f[0])
        if not len(data_output_list) == 6:
            self.loop_delay += 1
            return self. data_output_list_last
        if math.isnan(data_output_list[0]) or\
            math.isnan(data_output_list[1]) or\
            math.isnan(data_output_list[2]) or\
            math.isnan(data_output_list[3]) or\
            math.isnan(data_output_list[4]) or\
            math.isnan(data_output_list[5]):
            self.loop_delay += 1
            return self.data_output_list_last
        self. data_output_list_last = data_output_list
        self.loop_delay = 0
        return data_output_list



