from telnetlib import Telnet
import time
import sys

import struct

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
    return msp_set_raw_rc(throttle=0, aux4=1500)

def disarm():
    return msp_set_raw_rc(aux4=900)

def takeoff():
    return make_in(0xd9, struct.pack("<H", 1))

print(takeoff())

def land():
    return make_in(0xd9, struct.pack("<H", 2))

# print(arm())
# print(disarm())

with Telnet('192.168.4.1', 23) as tn:
    try:
        # tn.write(disarm())
        # sys.exit(0)
        tn.write(msp_set_raw_rc(aux4=1000))
        tn.write(arm())
        for i in range(50):
            if i < 10:
                tn.write(msp_set_raw_rc(throttle=1000))
            elif i == 10:
                tn.write(takeoff())
            # elif i < 20:
                # tn.write(msp_set_raw_rc(throttle=1800))
            elif i < 40:
                tn.write(msp_set_raw_rc(throttle=0))
            else:
                tn.write(land())

            print(i)
            time.sleep(0.2)
    except KeyboardInterrupt:
        tn.write(disarm())
        print('bye')
        sys.exit(0)

sys.exit(0)

with Telnet('192.168.4.1', 23) as tn:
    print('arm')
    tn.write(arm())
    time.sleep(3)

    print('takeoff')
    # tn.write(takeoff())
    # time.sleep(3)

    print('throttle 1700')
    tn.write(msp_set_raw_rc(throttle=1700))
    time.sleep(3)

    print('land')
    tn.write(land())
    time.sleep(3)

    tn.write(disarm())
    pass

# with Telnet('192.168.4.1', 23) as tn:
    # tn.write(msp_set_raw_rc(aux3=1500))
    # tn.write(arm())
    # print('a')
    # time.sleep(3)
    # tn.write(msp_set_raw_rc(throttle=1700))
    # print('b')
    # time.sleep(3)
    # tn.write(msp_set_raw_rc(throttle=1200))
    # print('d')
    # time.sleep(3)
    # tn.write(disarm())
    # pass
