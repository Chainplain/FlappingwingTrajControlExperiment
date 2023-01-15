###"""chainplan Rotation Computation,"""
# 2022 11-18
import numpy as np

EXTREME_SMALL_NUMBER_4_FILTER = 0.00000001
class Low_Pass_Second_Order_Filter():
    def __init__(self, initial_value, stop_freq, damping_rate, time_gap):
        self.y = initial_value
        self.y_minus_1 = initial_value
        self.y_minus_2 = initial_value
        self.input_minus_1 = initial_value
        self.input_minus_2 = initial_value

        self.omega_n = 2 * np.pi * stop_freq
        self.zeta = damping_rate
        self.filter_gap = time_gap

    def march_forward(self, input):
        A = 2.0 * (self.zeta * self.omega_n * self.filter_gap - 1)
        B = 1.0 - 2.0 * self.zeta * self.omega_n * self.filter_gap + \
            self.omega_n * self.filter_gap * self.omega_n * self.filter_gap
        C = self.omega_n * self.filter_gap * self.omega_n * self.filter_gap
        # self.y = C * self. input_minus_2 - A * self. y_minus_1 - B *  self. y_minus_2
        # print('A', A, 'B', B, 'C', C)
        self.y = C * self.input_minus_2 - A * self.y_minus_1 - B * self.y_minus_2
        # print('self.u : ', input[0, 0], self.input_minus_1[0, 0], self.input_minus_2[0, 0])
        # print('self.y : ', self.y[0, 0], self.y_minus_1[0, 0], self.y_minus_2[0, 0])
        self.input_minus_2 = self.input_minus_1
        self.input_minus_1 = input
        self.y_minus_2 = self.y_minus_1
        self.y_minus_1 = self.y

    def Get_filtered(self):
        return self.y

class FIR_Filter():
    def __init__(self, initial_value, FIR_weights, time_gap):
        self.y = initial_value
        self. weights = 1 / (np.sum(FIR_weights) + EXTREME_SMALL_NUMBER_4_FILTER) * np.array(FIR_weights)
        print('FIR_Filter_weights:', self. weights)
        self. old_list = []
        self. list_length = len(FIR_weights)
        for i in range( self. list_length):
            self.old_list.append( initial_value )
        self.filter_gap = time_gap

    def march_forward(self, input):
        for i in range( 1, self. list_length ) :
            self.old_list[self. list_length - i] = self.old_list[self. list_length - i - 1]
        self.old_list[0] = input

    def Get_filtered(self):
        filtered = 0 * self.y
        for i in range( len(self. old_list) ):
            filtered = filtered + self. weights[i] *  self.old_list[i]
        self.y = filtered
        return self.y