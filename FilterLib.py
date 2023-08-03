###"""chainplan Rotation Computation,"""
# 2022 11-18
import numpy as np
import math

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
        self.delta_y = initial_value

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
        self.delta_y = self.y - self.y_minus_1
        self.y_minus_2 = self.y_minus_1
        self.y_minus_1 = self.y

    def Get_filtered(self):
        return self.y

    def Get_filtered_D(self):
        return self.delta_y

###"""chainplan Rotation Computation,"""
# 2023 2-10 novel Flap filters

class FIR_Filter():
    def __init__(self, initial_value, FIR_weights, time_gap):
        self.y = initial_value
        self.weights = 1 / (np.sum(FIR_weights) + EXTREME_SMALL_NUMBER_4_FILTER) * np.array(FIR_weights)
        print('FIR_Filter_weights:', self.weights)
        self.old_list = []
        self.list_length = len(FIR_weights)
        for i in range(self.list_length):
            self.old_list.append(initial_value)
        self.filter_gap = time_gap

    def march_forward(self, input):
        for i in range(1, self.list_length):
            self.old_list[self.list_length - i] = self.old_list[self.list_length - i - 1]
        self.old_list[0] = input

    def Get_filtered(self):
        filtered = 0 * self.y
        for i in range(len(self.old_list)):
            filtered = filtered + self.weights[i] * self.old_list[i]
        self.y = filtered
        return self.y


def Omega(input_omega_in_mat):
    omega_x = input_omega_in_mat[0, 0]
    omega_y = input_omega_in_mat[1, 0]
    omega_z = input_omega_in_mat[2, 0]

    Omega_mat = np.mat([[0, -omega_x, -omega_y, -omega_z],
                        [omega_x, 0, omega_z, -omega_y],
                        [omega_y, -omega_z, 0, omega_x],
                        [omega_z, omega_y, -omega_x, 0]])
    return Omega_mat


def Quat_product(q_f, q_b):
    q_0 = q_f[0, 0]
    q_1 = q_f[1, 0]
    q_2 = q_f[2, 0]
    q_3 = q_f[3, 0]
    M_q = np.mat([[q_0, -q_1, -q_2, -q_3],
                  [q_1, q_0, -q_3, -q_2],
                  [q_2, q_3, q_0, -q_1],
                  [q_3, -q_2, q_1, q_0]])
    q_pro = M_q * q_b
    return q_pro


def Quat_dual(q_input):
    q_0 = q_input[0, 0]
    q_1 = q_input[1, 0]
    q_2 = q_input[2, 0]
    q_3 = q_input[3, 0]
    return np.mat([q_0, q_1, q_2, q_3]).T


class Mov_Ave_Filter():
    def __init__(self, Total_Length_max, dim):
        self.Initial_zeros = np.mat(np.zeros(dim)).T
        self.Data_list = []
        self.Length_max = Total_Length_max
        for i in range(Total_Length_max):
            self.Data_list.append(self.Initial_zeros)

    def march_forward(self, input):
        for i in range(1, self.Length_max):
            self.Data_list[self.Length_max - i] = self.Data_list[self.Length_max - i - 1]
        self.Data_list[0] = input

    def Get_filterd(self, Length):
        if (Length > self.Length_max - 1):
            Length = self.Length_max - 1
        Sum = 0.0
        for i in range(Length):
            Sum = Sum + self.Data_list[i]
        return Sum / Length


class Ave_Com_MARG_Filter():
    def __init__(self, Beta_g, Beta_f, Beta_W, Mu_0, Alpha_a, Alpha_m, mag_N, mag_U,
                 delta_t, Mov_ave_Fltr_len, k_f_initial, k_f_Min, k_f_Max):
        ## Here delta_t is the
        self.beta_g = Beta_g
        self.beta_f = Beta_f
        self.beta_W = Beta_W
        self.mu_0 = Mu_0
        self.alpha_a = Alpha_a
        self.alpha_m = Alpha_m

        self.q_est = np.mat([1, 0, 0, 0]).T
        self.Gyro_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)
        self.Accel_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)
        self.Mag_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)

        self.k_f_est = k_f_initial
        self.b_g_est = np.mat([0, 0, 0]).T
        self.step = delta_t

        mag_norm = np.linalg.norm([mag_N, mag_U]) + EXTREME_SMALL_NUMBER_4_FILTER
        self.m_N = mag_N / mag_norm
        self.m_U = mag_U / mag_norm

        self.k_f_min = k_f_Min
        self.k_f_max = k_f_Max

    def march_forward(self, omega_m_in_mat, a_m_in_mat, m_m_in_mat, f_F):
        self.Gyro_mov_fltr.march_forward(omega_m_in_mat)
        self.Accel_mov_fltr.march_forward(a_m_in_mat)
        self.Mag_mov_fltr.march_forward(m_m_in_mat)

        T_F = int(np.round(1 / f_F / self.step))
        # print('T_F:',T_F)
        omega_ave = self.Gyro_mov_fltr.Get_filterd(T_F)
        accel_ave = self.Accel_mov_fltr.Get_filterd(T_F)
        mag_ave = self.Mag_mov_fltr.Get_filterd(T_F)
        mag_ave = mag_ave / (np.linalg.norm(mag_ave) + EXTREME_SMALL_NUMBER_4_FILTER)

        # a_v_est = np.mat ([0, 0, -self. k_f_est * f_F]).T
        a_v_est = np.mat([0, 0, self.k_f_est * f_F]).T

        a_m_est_raw = accel_ave + a_v_est
        # print('accel_ave:',accel_ave)
        # print('a_m_est_raw:',a_m_est_raw)
        a_m_est = a_m_est_raw / (np.linalg.norm(a_m_est_raw) + EXTREME_SMALL_NUMBER_4_FILTER)

        delta_q_g_est = (0.5 * Omega(omega_ave - self.b_g_est) -
                         0.125 * Omega(omega_ave - self.b_g_est) * Omega(omega_ave - self.b_g_est)) * \
                        self.q_est * self.step

        eta_est = self.q_est[0, 0]
        epsilon_1_est = self.q_est[1, 0]
        epsilon_2_est = self.q_est[2, 0]
        epsilon_3_est = self.q_est[3, 0]

        J_a = np.mat([[-2 * epsilon_2_est, 2 * epsilon_3_est, -2 * eta_est, 2 * epsilon_1_est],
                      [2 * epsilon_1_est, 2 * eta_est, 2 * epsilon_3_est, 2 * epsilon_2_est],
                      [0, -4 * epsilon_1_est, -4 * epsilon_2_est, 0]])

        f_a = np.mat([[2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) - a_m_est[0, 0]],
                      [2 * (eta_est * epsilon_1_est + epsilon_2_est * epsilon_3_est) - a_m_est[1, 0]],
                      [1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est - a_m_est[2, 0]]])

        J_m_1_1 = 2 * epsilon_3_est * self.m_N - 2 * epsilon_2_est * self.m_U
        J_m_1_2 = 2 * epsilon_2_est * self.m_N + 2 * epsilon_3_est * self.m_U
        J_m_1_3 = 2 * epsilon_1_est * self.m_N - 2 * eta_est * self.m_U
        J_m_1_4 = 2 * eta_est * self.m_N + 2 * epsilon_1_est * self.m_U

        J_m_2_1 = 2 * epsilon_1_est * self.m_U
        J_m_2_2 = - 4 * epsilon_1_est * self.m_N + 2 * eta_est * self.m_U
        J_m_2_3 = 2 * epsilon_3_est * self.m_U
        J_m_2_4 = - 4 * epsilon_3_est * self.m_N + 2 * epsilon_2_est * self.m_U

        J_m_3_1 = - 2 * epsilon_1_est * self.m_N
        J_m_3_2 = - 2 * eta_est * self.m_N - 4 * epsilon_1_est * self.m_U
        J_m_3_3 = 2 * epsilon_3_est * self.m_N - 4 * epsilon_2_est * self.m_U
        J_m_3_4 = 2 * epsilon_2_est * self.m_N

        J_m = np.mat([[J_m_1_1, J_m_1_2, J_m_1_3, J_m_1_4],
                      [J_m_2_1, J_m_2_2, J_m_2_3, J_m_2_4],
                      [J_m_3_1, J_m_3_2, J_m_3_3, J_m_3_4]])

        f_m_1 = 2 * (epsilon_1_est * epsilon_2_est + eta_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) * self.m_U - mag_ave[0, 0]
        f_m_2 = (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_3_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_2_est * epsilon_3_est + eta_est * epsilon_1_est) * self.m_U - mag_ave[1, 0]
        f_m_3 = 2 * (epsilon_2_est * epsilon_3_est - eta_est * epsilon_1_est) * self.m_N + \
                (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est) * self.m_U - mag_ave[2, 0]

        f_m = np.mat([[f_m_1],
                      [f_m_2],
                      [f_m_3]])

        delta_q_W_est = - self.alpha_a * J_a.T * f_a - self.alpha_m * J_m.T * f_m

        mu_k = np.linalg.norm(delta_q_g_est)

        delta_b_g_est = - 2 * self.beta_g * (self.mu_0 + mu_k) * Quat_product(Quat_dual(self.q_est), delta_q_W_est)
        self.b_g_est = self.b_g_est + delta_b_g_est[1:4]
        # print('b_g_est:', self.b_g_est)

        f_az = 1 - 2 * self.q_est[1, 0] * self.q_est[1, 0] * self.q_est[1, 0] * self.q_est[2, 0] - \
               (accel_ave[2, 0] + self.k_f_est * f_F) / (
                           np.linalg.norm(accel_ave + a_v_est) + EXTREME_SMALL_NUMBER_4_FILTER)
        delta_k_f_est = - self.beta_f * (self.mu_0 + mu_k) * f_az * f_F / (
                    np.linalg.norm(accel_ave + a_v_est) + EXTREME_SMALL_NUMBER_4_FILTER)

        self.k_f_est = self.k_f_est + delta_k_f_est

        if self.k_f_est > self.k_f_max:
            self.k_f_est = self.k_f_max

        if self.k_f_est < self.k_f_min:
            self.k_f_est = self.k_f_min

        delta_q_k_est = delta_q_g_est + self.beta_W * (self.mu_0 + mu_k) * delta_q_W_est
        q_est_raw = self.q_est + delta_q_k_est
        self.q_est = q_est_raw / (np.linalg.norm(q_est_raw) + EXTREME_SMALL_NUMBER_4_FILTER)

    def Get_filtered_quat(self):
        return self.q_est


class Com_MARG_Filter():
    def __init__(self, Beta_g, Beta_f, Beta_W, Mu_0, Alpha_a, Alpha_m, mag_N, mag_U,
                 delta_t, Mov_ave_Fltr_len, k_f_initial, k_f_Min, k_f_Max):
        ## Here delta_t is the
        self.beta_g = Beta_g
        self.beta_f = Beta_f
        self.beta_W = Beta_W
        self.mu_0 = Mu_0
        self.alpha_a = Alpha_a
        self.alpha_m = Alpha_m

        self.q_est = np.mat([1, 0, 0, 0]).T
        self.Gyro_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)
        self.Accel_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)
        self.Mag_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)

        self.k_f_est = k_f_initial
        self.b_g_est = np.mat([0, 0, 0]).T
        self.step = delta_t

        mag_norm = np.linalg.norm([mag_N, mag_U]) + EXTREME_SMALL_NUMBER_4_FILTER
        self.m_N = mag_N / mag_norm
        self.m_U = mag_U / mag_norm

        self.k_f_min = k_f_Min
        self.k_f_max = k_f_Max

    def march_forward(self, omega_m_in_mat, a_m_in_mat, m_m_in_mat):
        # self. Gyro_mov_fltr.march_forward(omega_m_in_mat)
        # self. Accel_mov_fltr.march_forward(a_m_in_mat)
        # self. Mag_mov_fltr.march_forward(m_m_in_mat)

        # T_F = int(np.round(1 / f_F / self. step))
        # print('T_F:',T_F)
        omega_ave = omega_m_in_mat
        accel_ave = a_m_in_mat
        mag_ave = m_m_in_mat
        mag_ave = mag_ave / (np.linalg.norm(mag_ave) + EXTREME_SMALL_NUMBER_4_FILTER)

        a_v_est = 0

        a_m_est_raw = accel_ave
        a_m_est = a_m_est_raw / (np.linalg.norm(a_m_est_raw) + EXTREME_SMALL_NUMBER_4_FILTER)

        delta_q_g_est = (0.5 * Omega(omega_ave) -
                         0.125 * Omega(omega_ave) * Omega(omega_ave)) * \
                        self.q_est * self.step

        eta_est = self.q_est[0, 0]
        epsilon_1_est = self.q_est[1, 0]
        epsilon_2_est = self.q_est[2, 0]
        epsilon_3_est = self.q_est[3, 0]

        J_a = np.mat([[-2 * epsilon_2_est, 2 * epsilon_3_est, -2 * eta_est, 2 * epsilon_1_est],
                      [2 * epsilon_1_est, 2 * eta_est, 2 * epsilon_3_est, 2 * epsilon_2_est],
                      [0, -4 * epsilon_1_est, -4 * epsilon_2_est, 0]])

        f_a = np.mat([[2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) - a_m_est[0, 0]],
                      [2 * (eta_est * epsilon_1_est + epsilon_2_est * epsilon_3_est) - a_m_est[1, 0]],
                      [1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est - a_m_est[2, 0]]])

        J_m_1_1 = 2 * epsilon_3_est * self.m_N - 2 * epsilon_2_est * self.m_U
        J_m_1_2 = 2 * epsilon_2_est * self.m_N + 2 * epsilon_3_est * self.m_U
        J_m_1_3 = 2 * epsilon_1_est * self.m_N - 2 * eta_est * self.m_U
        J_m_1_4 = 2 * eta_est * self.m_N + 2 * epsilon_1_est * self.m_U

        J_m_2_1 = 2 * epsilon_1_est * self.m_U
        J_m_2_2 = - 4 * epsilon_1_est * self.m_N + 2 * eta_est * self.m_U
        J_m_2_3 = 2 * epsilon_3_est * self.m_U
        J_m_2_4 = - 4 * epsilon_3_est * self.m_N + 2 * epsilon_2_est * self.m_U

        J_m_3_1 = - 2 * epsilon_1_est * self.m_N
        J_m_3_2 = - 2 * eta_est * self.m_N - 4 * epsilon_1_est * self.m_U
        J_m_3_3 = 2 * epsilon_3_est * self.m_N - 4 * epsilon_2_est * self.m_U
        J_m_3_4 = 2 * epsilon_2_est * self.m_N

        J_m = np.mat([[J_m_1_1, J_m_1_2, J_m_1_3, J_m_1_4],
                      [J_m_2_1, J_m_2_2, J_m_2_3, J_m_2_4],
                      [J_m_3_1, J_m_3_2, J_m_3_3, J_m_3_4]])

        f_m_1 = 2 * (epsilon_1_est * epsilon_2_est + eta_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) * self.m_U - mag_ave[0, 0]
        f_m_2 = (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_3_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_2_est * epsilon_3_est + eta_est * epsilon_1_est) * self.m_U - mag_ave[1, 0]
        f_m_3 = 2 * (epsilon_2_est * epsilon_3_est - eta_est * epsilon_1_est) * self.m_N + \
                (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est) * self.m_U - mag_ave[2, 0]

        f_m = np.mat([[f_m_1],
                      [f_m_2],
                      [f_m_3]])

        delta_q_W_est = - self.alpha_a * J_a.T * f_a - self.alpha_m * J_m.T * f_m

        mu_k = np.linalg.norm(delta_q_g_est)

        # delta_b_g_est = - 2 * self.beta_g * ( self. mu_0 + mu_k) * Quat_product(Quat_dual( self.q_est), delta_q_W_est)
        # self.b_g_est  = self.b_g_est + delta_b_g_est[1:4]

        delta_q_k_est = delta_q_g_est + self.beta_W * (self.mu_0 + mu_k) * delta_q_W_est
        q_est_raw = self.q_est + delta_q_k_est
        self.q_est = q_est_raw / (np.linalg.norm(q_est_raw) + EXTREME_SMALL_NUMBER_4_FILTER)

    def Get_filtered_quat(self):
        return self.q_est


class Ave_2015ICRA_MARG_Filter():
    def __init__(self, Beta_g, Beta_f, Beta_W, Mu_0, Alpha_a, Alpha_m, mag_N, mag_U,
                 delta_t, Mov_ave_Fltr_len, k_f_initial, k_f_Min, k_f_Max):
        ## Here delta_t is the
        self.beta_g = Beta_g
        self.beta_f = Beta_f
        self.beta_W = Beta_W
        self.mu_0 = Mu_0
        self.alpha_a = Alpha_a
        self.alpha_m = Alpha_m

        self.q_est = np.mat([1, 0, 0, 0]).T
        self.Gyro_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)
        self.Accel_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)
        self.Mag_mov_fltr = Mov_Ave_Filter(Mov_ave_Fltr_len, 3)

        self.k_f_est = k_f_initial
        self.b_g_est = np.mat([0, 0, 0]).T
        self.step = delta_t

        mag_norm = np.linalg.norm([mag_N, mag_U]) + EXTREME_SMALL_NUMBER_4_FILTER
        self.m_N = mag_N / mag_norm
        self.m_U = mag_U / mag_norm

        self.k_f_min = k_f_Min
        self.k_f_max = k_f_Max

    def march_forward(self, omega_m_in_mat, a_m_in_mat, m_m_in_mat, f_F):
        self.Gyro_mov_fltr.march_forward(omega_m_in_mat)
        self.Accel_mov_fltr.march_forward(a_m_in_mat)
        self.Mag_mov_fltr.march_forward(m_m_in_mat)

        T_F = int(np.round(1 / f_F / self.step))
        print('T_F:', T_F)

        omega_ave = self.Gyro_mov_fltr.Get_filterd(T_F)
        accel_ave = self.Accel_mov_fltr.Get_filterd(T_F)
        mag_ave = self.Mag_mov_fltr.Get_filterd(T_F)
        mag_ave = mag_ave / (np.linalg.norm(mag_ave) + EXTREME_SMALL_NUMBER_4_FILTER)

        a_v_est = 0

        a_m_est_raw = accel_ave
        a_m_est = a_m_est_raw / (np.linalg.norm(a_m_est_raw) + EXTREME_SMALL_NUMBER_4_FILTER)

        delta_q_g_est = (0.5 * Omega(omega_ave)) * \
                        self.q_est * self.step

        eta_est = self.q_est[0, 0]
        epsilon_1_est = self.q_est[1, 0]
        epsilon_2_est = self.q_est[2, 0]
        epsilon_3_est = self.q_est[3, 0]

        J_a = np.mat([[-2 * epsilon_2_est, 2 * epsilon_3_est, -2 * eta_est, 2 * epsilon_1_est],
                      [2 * epsilon_1_est, 2 * eta_est, 2 * epsilon_3_est, 2 * epsilon_2_est],
                      [0, -4 * epsilon_1_est, -4 * epsilon_2_est, 0]])

        f_a = np.mat([[2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) - a_m_est[0, 0]],
                      [2 * (eta_est * epsilon_1_est + epsilon_2_est * epsilon_3_est) - a_m_est[1, 0]],
                      [1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est - a_m_est[2, 0]]])

        J_m_1_1 = 2 * epsilon_3_est * self.m_N - 2 * epsilon_2_est * self.m_U
        J_m_1_2 = 2 * epsilon_2_est * self.m_N + 2 * epsilon_3_est * self.m_U
        J_m_1_3 = 2 * epsilon_1_est * self.m_N - 2 * eta_est * self.m_U
        J_m_1_4 = 2 * eta_est * self.m_N + 2 * epsilon_1_est * self.m_U

        J_m_2_1 = 2 * epsilon_1_est * self.m_U
        J_m_2_2 = - 4 * epsilon_1_est * self.m_N + 2 * eta_est * self.m_U
        J_m_2_3 = 2 * epsilon_3_est * self.m_U
        J_m_2_4 = - 4 * epsilon_3_est * self.m_N + 2 * epsilon_2_est * self.m_U

        J_m_3_1 = - 2 * epsilon_1_est * self.m_N
        J_m_3_2 = - 2 * eta_est * self.m_N - 4 * epsilon_1_est * self.m_U
        J_m_3_3 = 2 * epsilon_3_est * self.m_N - 4 * epsilon_2_est * self.m_U
        J_m_3_4 = 2 * epsilon_2_est * self.m_N

        J_m = np.mat([[J_m_1_1, J_m_1_2, J_m_1_3, J_m_1_4],
                      [J_m_2_1, J_m_2_2, J_m_2_3, J_m_2_4],
                      [J_m_3_1, J_m_3_2, J_m_3_3, J_m_3_4]])

        f_m_1 = 2 * (epsilon_1_est * epsilon_2_est + eta_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) * self.m_U - mag_ave[0, 0]
        f_m_2 = (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_3_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_2_est * epsilon_3_est + eta_est * epsilon_1_est) * self.m_U - mag_ave[1, 0]
        f_m_3 = 2 * (epsilon_2_est * epsilon_3_est - eta_est * epsilon_1_est) * self.m_N + \
                (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est) * self.m_U - mag_ave[2, 0]

        f_m = np.mat([[f_m_1],
                      [f_m_2],
                      [f_m_3]])

        delta_q_W_est = - self.alpha_a * J_a.T * f_a - self.alpha_m * J_m.T * f_m

        # delta_b_g_est = - 2 * self.beta_g * ( self. mu_0 + mu_k) * Quat_product(Quat_dual( self.q_est), delta_q_W_est)
        # self.b_g_est  = self.b_g_est + delta_b_g_est[1:4]

        delta_q_k_est = delta_q_g_est + self.beta_W * (self.mu_0) * delta_q_W_est
        q_est_raw = self.q_est + delta_q_k_est
        self.q_est = q_est_raw / (np.linalg.norm(q_est_raw) + EXTREME_SMALL_NUMBER_4_FILTER)

    def Get_filtered_quat(self):
        return self.q_est


class EKF_MARG_FILTER():
    def __init__(self, sigma_G, sigma_A, simga_M, mag_N, mag_U, delta_t):
        self.sigma_g = sigma_G
        self.sigma_a = sigma_A
        self.sigma_m = simga_M
        self.m_N = mag_N
        self.m_U = mag_U

        self.q_est = np.mat([1, 0, 0, 0]).T
        self.P_k = 0.1 * np.mat(np.eye(4))
        self.step = delta_t

    def march_forward(self, omega_m_in_mat, a_m_in_mat, m_m_in_mat):
        omega_m_normed = omega_m_in_mat / (np.linalg.norm(omega_m_in_mat) + EXTREME_SMALL_NUMBER_4_FILTER)
        a_m_normed = a_m_in_mat / (np.linalg.norm(a_m_in_mat) + EXTREME_SMALL_NUMBER_4_FILTER)
        m_m_normed = m_m_in_mat / (np.linalg.norm(m_m_in_mat) + EXTREME_SMALL_NUMBER_4_FILTER)

        self.q_est = self.q_est + (0.5 * Omega(omega_m_normed)) * self.q_est * self.step
        F_k = np.mat(np.eye(4)) + 0.5 * Omega(omega_m_normed) * self.step

        Xi = np.mat([[self.q_est[1, 0], self.q_est[2, 0], self.q_est[3, 0]],
                     [-self.q_est[0, 0], self.q_est[3, 0], -self.q_est[2, 0]],
                     [-self.q_est[3, 0], -self.q_est[0, 0], self.q_est[1, 0]],
                     [self.q_est[2, 0], -self.q_est[1, 0], -self.q_est[0, 0]]])
        Sigma_omega = 0.25 * self.step * self.step * Xi * (self.sigma_g * np.mat(np.eye(3))) * Xi.T
        self.P_k = F_k * self.P_k * F_k.T + Sigma_omega

        eta_est = self.q_est[0, 0]
        epsilon_1_est = self.q_est[1, 0]
        epsilon_2_est = self.q_est[2, 0]
        epsilon_3_est = self.q_est[3, 0]

        J_a = np.mat([[-2 * epsilon_2_est, 2 * epsilon_3_est, -2 * eta_est, 2 * epsilon_1_est],
                      [2 * epsilon_1_est, 2 * eta_est, 2 * epsilon_3_est, 2 * epsilon_2_est],
                      [0, -4 * epsilon_1_est, -4 * epsilon_2_est, 0]])

        f_a = np.mat([[2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) - a_m_normed[0, 0]],
                      [2 * (eta_est * epsilon_1_est + epsilon_2_est * epsilon_3_est) - a_m_normed[1, 0]],
                      [1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est - a_m_normed[2, 0]]])

        J_m_1_1 = 2 * epsilon_3_est * self.m_N - 2 * epsilon_2_est * self.m_U
        J_m_1_2 = 2 * epsilon_2_est * self.m_N + 2 * epsilon_3_est * self.m_U
        J_m_1_3 = 2 * epsilon_1_est * self.m_N - 2 * eta_est * self.m_U
        J_m_1_4 = 2 * eta_est * self.m_N + 2 * epsilon_1_est * self.m_U

        J_m_2_1 = 2 * epsilon_1_est * self.m_U
        J_m_2_2 = - 4 * epsilon_1_est * self.m_N + 2 * eta_est * self.m_U
        J_m_2_3 = 2 * epsilon_3_est * self.m_U
        J_m_2_4 = - 4 * epsilon_3_est * self.m_N + 2 * epsilon_2_est * self.m_U

        J_m_3_1 = - 2 * epsilon_1_est * self.m_N
        J_m_3_2 = - 2 * eta_est * self.m_N - 4 * epsilon_1_est * self.m_U
        J_m_3_3 = 2 * epsilon_3_est * self.m_N - 4 * epsilon_2_est * self.m_U
        J_m_3_4 = 2 * epsilon_2_est * self.m_N

        J_m = np.mat([[J_m_1_1, J_m_1_2, J_m_1_3, J_m_1_4],
                      [J_m_2_1, J_m_2_2, J_m_2_3, J_m_2_4],
                      [J_m_3_1, J_m_3_2, J_m_3_3, J_m_3_4]])

        f_m_1 = 2 * (epsilon_1_est * epsilon_2_est + eta_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_1_est * epsilon_3_est - eta_est * epsilon_2_est) * self.m_U - m_m_normed[0, 0]
        f_m_2 = (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_3_est * epsilon_3_est) * self.m_N + \
                2 * (epsilon_2_est * epsilon_3_est + eta_est * epsilon_1_est) * self.m_U - m_m_normed[1, 0]
        f_m_3 = 2 * (epsilon_2_est * epsilon_3_est - eta_est * epsilon_1_est) * self.m_N + \
                (1 - 2 * epsilon_1_est * epsilon_1_est - 2 * epsilon_2_est * epsilon_2_est) * self.m_U - m_m_normed[
                    2, 0]

        f_m = np.mat([[f_m_1],
                      [f_m_2],
                      [f_m_3]])

        delta_q_W_est = -  J_a.T * f_a - J_m.T * f_m
        J = np.vstack((J_a, J_m)).T

        zeros_3_3 = np.zeros((3, 3))
        sigma_a_diag = self.sigma_a * np.eye(3)
        sigma_m_diag = self.sigma_m * np.eye(3)
        half_upper_v_k = np.hstack((sigma_a_diag, zeros_3_3))
        half_lower_v_k = np.hstack((zeros_3_3, sigma_m_diag))
        inner_v_k = np.vstack((half_upper_v_k, half_lower_v_k))

        Sigma_v_k = J * inner_v_k * J.T

        K = self.P_k * np.linalg.pinv(self.P_k + Sigma_v_k)
        # K = 0.01
        K_eig, K_vec = np.linalg.eig(K)

        self.q_est = self.q_est + 5 * max(abs(K_eig)) * delta_q_W_est

        self.q_est = self.q_est / (np.linalg.norm(self.q_est) + EXTREME_SMALL_NUMBER_4_FILTER)

        self.P_k = (np.eye(4) - K) * self.P_k

        print('K_eig :', K_eig)

    def Get_filtered_quat(self):
        return self.q_est


class OSC_LEARNER():
    def __init__(self, DF_stop_freq, DF_zeta_n, beta_T, K_x, K_q, delta_t, x_initial, mem_stack_len, delta_f_bound):
        self.df_stop_freq = DF_stop_freq
        self.df_zeta_n = DF_zeta_n
        self.period_learning_rate = beta_T
        self.os_learning_rate = K_x
        self.os_correction_rate = K_q

        self.x = x_initial
        self.delta_T_est = 0
        self.q_F_est = np.mat([1, 0, 0, 0]).T

        self.step = delta_t
        self.delta_T_bound = int(round(1 / delta_f_bound / self.step))

        self.x_m_MAF = Mov_Ave_Filter(mem_stack_len, x_initial.shape[1])
        self.q_F_MAF = Mov_Ave_Filter(mem_stack_len, 4)

        self.x_m_LPSF = Low_Pass_Second_Order_Filter(0 * x_initial, self.df_stop_freq, self.df_zeta_n, self.step)
        # paras: initial_value, stop_freq, damping_rate, time_gap

        # self. MS_len = mem_stack_len
        self.Data_list = []
        self.d_Data_list = []
        self.est_Data_list = []
        self.Length_max = mem_stack_len
        for i in range(2 * mem_stack_len):
            self.Data_list.append(0 * x_initial)
            self.d_Data_list.append(0 * x_initial)
            self.est_Data_list.append(0 * x_initial)

    def march_forward(self, gyro_meas, acc_meas, mag_meas, f_F):
        x_meas = np.hstack((gyro_meas.T, acc_meas.T, mag_meas.T)).T
        self.x_m_MAF.march_forward(x_meas)

        T_F = int(math.ceil(1 / f_F / self.step + self.delta_T_est))
        if T_F < 0:
            T_F = 0
        if T_F > self.Length_max - 1:
            T_F = self.Length_max - 1

        x_meas_ave = self.x_m_MAF.Get_filterd(T_F)
        self.x_m_LPSF.march_forward(x_meas - x_meas_ave)
        x_F = self.x_m_LPSF.Get_filtered()
        x_F_d = self.x_m_LPSF.Get_filtered_D()

        Delta_D_T = 0
        for i in range(self.Length_max):
            Delta_D_T = Delta_D_T + (self.Data_list[i + T_F] - self.Data_list[i]).T * \
                        self.d_Data_list[i + T_F] * self.step
        self.delta_T_est = self.delta_T_est + self.period_learning_rate * Delta_D_T

        if self.delta_T_est > self.delta_T_bound:
            self.delta_T_est = self.delta_T_bound
        if self.delta_T_est < -self.delta_T_bound:
            self.delta_T_est = -self.delta_T_bound

        ### data memory stack march forward
        for i in range(1, self.Length_max * 2):
            self.Data_list[self.Length_max * 2 - i] = self.Data_list[self.Length_max * 2 - i - 1]
            self.d_Data_list[self.Length_max * 2 - i] = self.d_Data_list[self.Length_max * 2 - i - 1]
        self.Data_list[0] = x_F
        self.d_Data_list[0] = x_F_d

        x_F_est = self.est_Data_list[T_F] + self.os_learning_rate * (x_F - self.est_Data_list[0])
        for i in range(1, self.Length_max * 2):
            self.est_Data_list[self.Length_max * 2 - i] = self.est_Data_list[self.Length_max * 2 - i - 1]
        self.est_Data_list[0] = x_F_est

        q_F_ave = self.q_F_MAF.Get_filterd(T_F)
        q_F_ave_normed = q_F_ave / (np.linalg.norm(q_F_ave) + EXTREME_SMALL_NUMBER_4_FILTER)

        omega_F_est = x_F_est[0:3, 0]
        q_F_ave_dual = np.mat(
            [q_F_ave_normed[0, 0], -q_F_ave_normed[1, 0], -q_F_ave_normed[2, 0], -q_F_ave_normed[3, 0]]).T

        omega_c = np.sign(q_F_ave_dual[0, 0]) * self.os_correction_rate * q_F_ave_dual[1:4]
        omega_add_tog = omega_F_est + omega_c
        print('omega_F_est N:', np.linalg.norm(omega_F_est))
        print('omega_c N:', np.linalg.norm(omega_c))

        self.q_F_est = self.q_F_est + (
                    0.5 * Omega(omega_add_tog) - 0.125 * Omega(omega_add_tog) * Omega(omega_add_tog)) * \
                       self.q_F_est * self.step
        self.q_F_est = self.q_F_est / (np.linalg.norm(self.q_F_est) + EXTREME_SMALL_NUMBER_4_FILTER)

        self.q_F_MAF.march_forward(self.q_F_est)

    def Get_learned_quat(self):
        return self.q_F_est

    def Get_dev_T(self):
        return self.step * self.delta_T_est



























