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

print(takeoff())

def land():
    return make_in(0xd9, struct.pack("<H", 2))

# print(arm())
# print(disarm())
last_button = None
def button_pressed(btn):
    global last_button
    last_button = str(btn.name)

with Telnet('192.168.4.1', 23) as tn, Xbox360Controller() as controller:
# with Xbox360Controller() as controller:
    controller.button_a.when_pressed = button_pressed
    controller.button_b.when_pressed = button_pressed
    controller.button_x.when_pressed = button_pressed
    controller.button_y.when_pressed = button_pressed

    armed = False
    throttle=1000

    try:
        while True:
            time.sleep(0.1)

            if last_button == "button_a":
                armed = False
            elif last_button == "button_b":
                armed = True
                throttle = 1000
            elif last_button == "button_x" and not armed:
                print('TAKEOFF START                                                                   ', end='\n', flush=True)
                tn.write(box_arm())
                tn.write(takeoff())
                time.sleep(2)
                armed = True
                throttle = 1600
            elif last_button == "button_y" and armed:
                print('LAND START                                                                      ', end='\n', flush=True)
                tn.write(land())
                time.sleep(2)
                throttle=1000
                armed = False
            last_button = None

            if not armed:
                tn.write(disarm())
                print('DISARMED                                                                        ', end='\n', flush=True)
                continue

            throttle += controller.axis_l.y * -10 # moving joystick up makes y negative
            if throttle < 1000:
                throttle = 1000
            elif throttle > 2100:
                throttle = 2100

            yaw = 1500 + int(controller.axis_l.x * 300)
            pitch = 1540 + int(controller.axis_r.y * -300) # moving joystick up makes y negative
            roll = 1500 + int(controller.axis_r.x * 300)

            print('throttle: {:4.2f}    roll: {:4}    pitch {:4}    yaw {:4}'.format(throttle, roll, pitch, yaw), end='\n', flush=True)
            tn.write(msp_set_raw_rc(roll=roll, pitch=pitch, throttle=int(throttle), yaw=yaw))

    except KeyboardInterrupt:
        pass
        tn.write(disarm())

    sys.exit(0)
