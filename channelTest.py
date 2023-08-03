import serial
from   SerialTRAN import Serial_transmit_multi_protocol as STM
import time

MultiProtocol_ser = serial.Serial(
        port='COM8',
        baudrate=100000,
        parity= serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )

max_PWM = 900
min_PWM = 2100
default_PWM = 1500

setting = [0, 21, 0, 1, 1, 0, 0, 0]
op = 6
Duration = 200
print('Warming up!!')

for channel in range(0,4):
    print('___________\n Now is channel ', channel)
    print('Low!!')
    for i in range(0, Duration):
        D = [default_PWM] * 16
        D[channel] = min_PWM
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)
        # print('\n')
    print('Gradually Changing.\n')
    for i in range(0, Duration):
        D = [default_PWM] * 16
        D[channel] = min_PWM + i / Duration * (max_PWM - min_PWM)
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)
    print('High!!')
    for i in range(0, Duration):
        D = [default_PWM] * 16
        D[channel] = max_PWM
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)
    print('Gradually Changing.\n')
    for i in range(0, Duration):
        D = [default_PWM] * 16
        D[channel] = max_PWM + i / Duration * (min_PWM - max_PWM)
        MultiProtocol_ser.write(bytes().fromhex(bytes(STM(setting, op, D)).hex()))
        time.sleep(0.01)