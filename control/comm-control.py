from telnetlib import Telnet
import time
import sys
import struct
from xbox360controller import Xbox360Controller

def make_in(command: int, byte_arr: bytes):

    cmd = struct.pack(f"<cBB{len(byte_arr)}s", b'<', len(byte_arr), command, byte_arr)

    crc = 0
    for c in cmd[1:]:
        crc ^= c
    crcb = bytes([crc])
    return b"$M" + cmd + crcb
    pass

def msp_set_raw_rc(roll=1500, pitch=1500, throttle=1000, yaw=1500, aux1=2100, aux2=900, aux3=1500, aux4=1500):
    payload = struct.pack("<8H", roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)
    return make_in(0xc8, payload)

def arm():
    return msp_set_raw_rc(throttle=1000, aux4=1500)

def box_arm():
    return msp_set_raw_rc(throttle=1500, aux4=1500)

def disarm():
    return msp_set_raw_rc(aux4=900)

def takeoff():
    return make_in(0xd9, struct.pack("<H", 1))

def land():
    return make_in(0xd9, struct.pack("<H", 2))

def acc_calib():
    return make_in(0xcd, b"")

def mag_calib():
    return make_in(0xce, b"")

# print(arm())
# print(disarm())
last_button = None
def button_pressed(btn):
    global last_button
    last_button = str(btn.name)

print('------')

with Telnet('192.168.4.1', 23) as tn, Xbox360Controller() as controller:
# with Xbox360Controller() as controller:
    controller.button_a.when_pressed = button_pressed
    controller.button_b.when_pressed = button_pressed
    controller.button_x.when_pressed = button_pressed
    controller.button_y.when_pressed = button_pressed

    armed = False

    # tn.write(disarm())
    # tn.write(acc_calib())
    # tn.write(mag_calib())

    try:
        while True:
            time.sleep(0.1)

            if last_button == "button_a":
                armed = False
            elif last_button == "button_b":
                tn.write(arm())
                armed = True
            elif last_button == "button_x" and not armed:
                print('TAKEOFF START                                                                   ', end='\n', flush=True)
                tn.write(box_arm())
                tn.write(takeoff())
                time.sleep(1)
                armed = True
            elif last_button == "button_y" and armed:
                print('LAND START                                                                      ', end='\n', flush=True)
                tn.write(land())
                time.sleep(5)
                armed = False
            last_button = None

            if not armed:
                tn.write(disarm())
                print('DISARMED                                                                        ', end='\n', flush=True)
                continue

            throttle = 1500
            ly = -controller.axis_l.y # minus because joystick up is -ve
            if ly > 0:
                throttle += int(ly * 200)
            else:
                # drone drops fast if throttle goes too low
                throttle += int(ly * 200)

            yaw = 1500 + int(controller.axis_l.x * 300)
            pitch = 1500 + int(controller.axis_r.y * -200) # moving joystick up makes y negative
            roll = 1500 + int(controller.axis_r.x * 200)

            print('throttle: {:4.2f}    roll: {:4}    pitch {:4}    yaw {:4}'.format(throttle, roll, pitch, yaw), end='\n', flush=True)
            tn.write(msp_set_raw_rc(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw))

    except KeyboardInterrupt:
        pass
        tn.write(disarm())

    sys.exit(0)
