import pygame
import sys
from telnetlib import Telnet
import time
import sys
import struct

pygame.init()
display = pygame.display.set_mode((300, 300))
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
    msg = make_in(0xc8, payload)
    print(msg)
    return msg

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

print('------')

with Telnet('192.168.4.1', 23) as tn:
    try:
        throttle=1500
        roll=1500
        pitch=1500
        yaw=1500
        armed= False
        while True:
            time.sleep(0.1)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        throttle = throttle + 100
                    if event.key == pygame.K_s:
                        throttle = throttle - 100
                    if event.key == pygame.K_a:
                        yaw = yaw + 50
                    if event.key == pygame.K_d:
                        yaw = yaw - 50
                    if event.key == pygame.K_UP:
                        pitch = pitch + 50
                    if event.key == pygame.K_DOWN:
                        pitch = pitch - 50
                    if event.key == pygame.K_LEFT:
                        roll = roll - 50
                    if event.key == pygame.K_RIGHT:
                        roll = roll + 50
                    if event.key == pygame.K_t:
                        print('TAKEOFF START                                                                   ', end='\n', flush=True)
                        tn.write(box_arm())
                        tn.write(takeoff())
                        time.sleep(1)
                        armed = True
                    if event.key == pygame.K_l:
                        print('LAND START                                                                      ', end='\n', flush=True)
                        tn.write(land())
                        time.sleep(5)
                        armed = False
                    if event.key == pygame.K_x:
                        armed = False
                    if event.key == pygame.K_e:
                        tn.write(arm())
                        armed = True
                    if not armed:
                        tn.write(disarm())
                        print('DISARMED                                                                        ', end='\n', flush=True)
                        continue

            print('throttle: {:4.2f}    roll: {:4}    pitch {:4}    yaw {:4}'.format(throttle, roll, pitch, yaw), end='\n', flush=True)
            tn.write(msp_set_raw_rc(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw))

    except KeyboardInterrupt:
        pass
        tn.write(disarm())

    sys.exit(0)