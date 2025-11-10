import gamepad
from cyberpi import mbot2
import time

mic_ser_r = 'S1'   # แขนขวา
mic_ser_l = 'S2'   # แขนซ้าย
dig_ser = 'S3'     # ที่หมุน/หนีบ

mic_ser_l_val = 0
mic_ser_r_val = 90
dig_ser_val = 0

mbot2.servo_set(mic_ser_l_val, mic_ser_l)
mbot2.servo_set(mic_ser_r_val, mic_ser_r)
mbot2.servo_set(dig_ser_val, dig_ser)

while True:
    x = gamepad.get_joystick('Lx')
    y = gamepad.get_joystick('Ly')
    mbot2.drive_power((y - x) * -1, (x + y))

    if gamepad.is_key_pressed('N2'):
        dig_ser_val += 2
        if dig_ser_val > 180:
            dig_ser_val = 180
        mbot2.servo_set(dig_ser_val, dig_ser)

    elif gamepad.is_key_pressed('N3'):
        dig_ser_val -= 5
        if dig_ser_val < 0:
            dig_ser_val = 0
        mbot2.servo_set(dig_ser_val, dig_ser)
        
    if gamepad.is_key_pressed('Up'):
        mbot2.servo_set(190, dig_ser) 
        time.sleep(1.1) 
        mbot2.servo_set(90, mic_ser_r)  # S1
        mbot2.servo_set(0, mic_ser_l)   # S2 
        time.sleep(0.5)

    if gamepad.is_key_pressed('N4'):
        mbot2.servo_set(148, dig_ser)
        time.sleep(0.2)

    if gamepad.is_key_pressed('L1'):
        mbot2.servo_set(0, mic_ser_r)
        mbot2.servo_set(90, mic_ser_l)

    if gamepad.is_key_pressed('R1'):
        mbot2.servo_set(90, mic_ser_r)
        mbot2.servo_set(0, mic_ser_l)

    time.sleep(0.05)