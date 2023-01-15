### Chainplain serial trans for multiprotocol
### 2023-1-5 17:41:31
import math

def Serial_transmit_multi_protocol(Settings, Option, DataTran):
    s0 = 85     # 0x55
    if Settings [0] :
        s0 = 84 # 0x54

    s1_1 = Settings[1] # 0x1F protocal2
    if Settings[2] :
        s1_2 = 32 # 0x20      range
    else :
        s1_2 = 0

    if Settings[3] :
        s1_3 = 64 # 0x40 autobind
    else :
        s1_3 = 0

    if Settings[4] :
        s1_4 = 128 # 0x80 bind
    else :
        s1_4 = 0

    s1 = s1_1 | s1_2 | s1_3 | s1_4     #$bitor(bitor(S1_1, S1_2), bitor(S1_3, S1_4));

    if Settings[5] :
        s2_1 = 128 # 0x80 power
    else :
        s2_1 = 0

    s2_2 = (Settings[6] * 16) & 112 # 0x70 sub - pro


    s2_3 = Settings[7] & 15  # 0x0F Rx - num


    s2 = s2_1 | s2_2 | s2_3

    s3 = Option

    length_of_DataTran = len( DataTran )

    for i in range(0, length_of_DataTran) :
        if DataTran [i] < 860 :
            DataTran [i] = 860
        if DataTran [i] > 2140 :
            DataTran [i] = 2140

    data_bin = [0] * length_of_DataTran

    for i in range(0, length_of_DataTran):
        data_bin [i] = 2047 & math.floor( (DataTran [i] - 860) /5 * 8 )

    s4_25 = [0] * 22

    s4_25[0] = data_bin [0] & 255
    s4_25[1] =( (data_bin [0] & 1792) >> 8 ) | ( (data_bin [1] & 31) << 3 )#bitor(bitshift(bitand(Data_bin(1), 2048 - 256), -8), bitshift(bitand(Data_bin(2), 31), 3));
    s4_25[2] =( (data_bin [1] & 2016) >> 5 )  | ( (data_bin [2] & 3 ) << 6 )#bitor(bitshift(bitand(Data_bin(2), 2048 - 32), -5), bitshift(bitand(Data_bin(3), 3), 6));
    s4_25[3] =( (data_bin [2] & 2044) >> 2 )  & 255 #bitand(bitshift(bitand(Data_bin(3), 2048 - 4), -2), 255);
    s4_25[4] =( (data_bin [2] & 1024) >> 10)  | ( (data_bin[3] & 127) << 1 )#bitor(bitshift(bitand(Data_bin(3), 2048 - 1024), -10), bitshift(bitand(Data_bin(4), 127), 1));
    s4_25[5] =( (data_bin [3] & 1920) >> 7)  | ( (data_bin[4] & 15) << 4 )#bitor(bitshift(bitand(Data_bin(4), 2048 - 128), -7), bitshift(bitand(Data_bin(5), 15), 4));
    s4_25[6] =( (data_bin [4] & 2032) >> 4)  | ( (data_bin[5] & 1) << 7 )  #bitor(bitshift(bitand(Data_bin(5), 2048 - 16), -4), bitshift(bitand(Data_bin(6), 1), 7));
    s4_25[7] =( (data_bin [5] & 2046) >> 1 )  & 255 #bitand(bitshift(bitand(Data_bin(6), 2048 - 2 ^ 1), -1), 255);
    s4_25[8] =( (data_bin [5] & 1536) >> 9)  | ( (data_bin[6] & 63) << 2 )  #bitor(bitshift(bitand(Data_bin(6), 2048 - 512), -9), bitshift(bitand(Data_bin(7), 63), 2));
    s4_25[9] =( (data_bin [6] & 1984) >> 6)  | ( (data_bin[7] & 7) << 5 ) #bitor(bitshift(bitand(Data_bin(7), 2048 - 64), -6), bitshift(bitand(Data_bin(8), 7), 5));
    s4_25[10] =( (data_bin[7] & 2040) >> 3) & 255 #bitand(bitshift(bitand(Data_bin(8), 2048 - 8), -3), 255);
    s4_25[11] = data_bin[8] & 255 #bitand(Data_bin(9), 255);
    s4_25[12] = ( (data_bin [8] & 1792) >> 8)  | ( (data_bin[9] & 31) << 3 ) #bitor(bitshift(bitand(Data_bin(9), 2048 - 256), -8), bitshift(bitand(Data_bin(10), 31), 3));
    s4_25[13] = ( (data_bin [9] & 2016) >> 5)  | ( (data_bin[10] & 3) << 6 ) #bitor(bitshift(bitand(Data_bin(10), 2048 - 32), -5), bitshift(bitand(Data_bin(11), 3), 6));
    s4_25[14] = ( (data_bin[10] & 2044) >> 2) & 255 #bitand(bitshift(bitand(Data_bin(11), 2048 - 4), -2), 255);
    s4_25[15] = ( (data_bin [10] & 1024) >> 10)  | ( (data_bin[11] & 127) << 1 ) #bitor(bitshift(bitand(Data_bin(11), 2048 - 1024), -10), bitshift(bitand(Data_bin(12), 127), 1));
    s4_25[16] = ( (data_bin [11] & 1920) >> 7)  | ( (data_bin[12] & 15) << 4 ) #bitor(bitshift(bitand(Data_bin(12), 2048 - 128), -7), bitshift(bitand(Data_bin(13), 15), 4));
    s4_25[17] = ( (data_bin [12] & 2032) >> 4)  | ( (data_bin[13] & 1) << 7 )#bitor(bitshift(bitand(Data_bin(13), 2048 - 16), -4), bitshift(bitand(Data_bin(14), 1), 7));
    s4_25[18] = ( (data_bin[13] & 2046) >> 1) & 255 #bitand(bitshift(bitand(Data_bin(14), 2048 - 2 ^ 1), -1), 255);
    s4_25[19] = ( (data_bin [13] & 1536) >> 9)  | ( (data_bin[14] & 63) << 2 )#bitor(bitshift(bitand(Data_bin(14), 2048 - 512), -9), bitshift(bitand(Data_bin(15), 63), 2));
    s4_25[20] = ( (data_bin [14] & 1984) >> 6)  | ( (data_bin[15] & 7) << 5 )#bitor(bitshift(bitand(Data_bin(15), 2048 - 64), -6), bitshift(bitand(Data_bin(16), 7), 5));
    s4_25[21] = ( (data_bin[15] & 2040) >> 3) & 255 #bitand(bitshift(bitand(Data_bin(16), 2048 - 8), -3), 255);

    s_tran = s4_25
    s_tran.insert(0, s3)
    s_tran.insert(0, s2)
    s_tran.insert(0, s1)
    s_tran.insert(0, s0)

    # print(s_tran)

    return s_tran

