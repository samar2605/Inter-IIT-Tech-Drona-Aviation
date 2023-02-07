import struct
import socket
import time
import sys

# parses an MSP packet from the drone
# returns a tuple of (command, payload)
def parse_out(msg):
    if msg[:3] != b'$M>':
        raise ValueError("Invalid message header/direction")

    length, command = struct.unpack("<BB", msg[3:5])
    if len(msg[5:-1]) != length:
        raise ValueError("Wrong packet length")

    payload = msg[5:5+length]

    # verify crc
    # crc is xor of length, command, and each byte in payload
    crc = length ^ command
    for c in payload:
        crc ^= c
    if crc != msg[5+length]: # crc is last byte, after 3 header bytes, length, command, and payload
        raise ValueError("Mismatched crc")

    return command, payload

# builds an MSP packet to send to the drone, given a command and payload
def make_in(command: int, byte_arr: bytes):

    cmd = struct.pack(f"<cBB{len(byte_arr)}s", b'<', len(byte_arr), command, byte_arr)

    # calculate crc
    crc = 0
    for c in cmd[1:]:
        crc ^= c
    crcb = bytes([crc])
    return b"$M" + cmd + crcb

# builds an MSP_SET_RAW_RC packet
def msp_set_raw_rc(roll=1500, pitch=1500, throttle=1000, yaw=1500, aux1=2100, aux2=900, aux3=1500, aux4=1500):
    payload = struct.pack("<8H", roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)
    return make_in(0xc8, payload)

# builds an MSP_SET_COMMAND packet
def msp_set_command(command):
    payload = struct.pack("<H", command)
    return make_in(0xd9, payload)

# MSP_ACC_CALIBRATION and MSP_MAG_CALIBRATION
ACC_CALIB = make_in(0xcd, b"")
MAG_CALIB = make_in(0xce, b"")

class Command:
    def __init__(self, ip_addr, interface=None):
        # connect to drone
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if interface is not None:
            if not sys.platform.startswith("linux"):
                raise RuntimeError("can only specify interface on linux")
            # bind to specific interface
            # SO_BINDTODEVICE is not part of the standard socket module (its not portable)
            # so we have to use the numeric value
            SO_BINDTODEVICE = 25
            # C interface requires null-terminated string
            interface = interface.encode() + b'\0'
            self.socket.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, interface)
        self.socket.connect((ip_addr, 23))

        # read 1024 bytes from socket, with timeout of 2 seconds
        self.socket.settimeout(1)
        try:
            self.socket.recv(1024)
        except:
            pass
        self.socket.settimeout(None)

        self.throttle = 1500
        self.yaw = 1500
        self.pitch = 1500
        self.roll = 1500

        # TODO: is 1500 for aux1/aux2 fine?
        self.aux1 = 2100
        self.aux2 = 900
        self.aux3 = 1500
        self.aux4 = 900

        # Xbox controller
        self.controller = None
        self.lastButton = None

        # if zero, command is of type MSP_SET_RAW_RC
        # if non-zero, self.cmd is the payload for MSP_SET_COMMAND
        #   1 is takeoff, 2 is land
        self.cmd = 0

    def __del__(self):
        # disconnect from drone when object is destroyed
        self.socket.close()

    def is_armed(self):
        return self.cmd == 0 and 1300 < self.aux4 < 1700

    # ------------------- MSP commands -------------------

    def get_raw_vals(self):
        MSP_RAW_IMU = 0x66
        data = self.get_in_msg(MSP_RAW_IMU, 18)
        arr = struct.unpack("<9h", data)
        return {
            "acc": arr[:3],
            "gyro": arr[3:6],
            "mag": arr[6:]
        }

    def set_attitude(self, throttle=None, yaw=None, pitch=None, roll=None):
        self.cmd = 0
        self.throttle = throttle or self.throttle
        self.yaw = yaw or self.yaw
        self.pitch = pitch or self.pitch
        self.roll = roll or self.roll
        self.send()

    def disarm(self):
        self.cmd = 0
        self.throttle = 1000
        self.aux4 = 900
        self.send()

    def arm(self):
        self.cmd = 0
        self.throttle = 1000
        self.yaw = 1500
        self.pitch = 1500
        self.roll = 1500
        self.aux4 = 1500
        self.send()

    def boxarm(self):
        self.cmd = 0
        self.throttle = 1500
        self.yaw = 1500
        self.pitch = 1500
        self.roll = 1500
        self.aux4 = 1500
        self.send()

    # execute the takeoff sequence (disarm, boxarm, takeoff), if the drone is not armed
    def takeoff(self):
        if self.is_armed():
            # already armed => could already be in the air => don't try to takeoff again
            return

        self.disarm()
        time.sleep(0.1)

        self.boxarm()
        self.cmd = 1
        self.send()

        # time.sleep(2)
        self.boxarm()

    def land(self):
        self.cmd = 2
        self.send()
        time.sleep(5)
        self.disarm()

    # calibrate the accelerometer and magnetometer, if the drone is not armed
    def calib(self):
        if self.is_armed():
            return
        self.send_out_msg(ACC_CALIB)
        self.send_out_msg(MAG_CALIB)



    # ------------------- Communication -------------------

    # create a MSP packet using the current command
    def make_msg(self):
        if self.cmd == 0:
            return msp_set_raw_rc(self.roll, self.pitch, self.throttle, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4)
        else:
            return msp_set_command(self.cmd)

    # send the current command to the drone
    # updates attitude values from the controller, if there is a controller connected
    def send(self):
        if self.is_armed() and self.controller is not None:
            self.read_controller_axes()
        self.send_out_msg(self.make_msg())

    # send an OUT message to the drone, and read the response (empty acknowledgement packet)
    # used by:
    # - send() to send a command
    # - calib() to calibrate the accelerometer and magnetometer
    def send_out_msg(self, msg):
        self.send_msg(msg)
        self.get_msg(0)

    # send a message to the drone
    # used by:
    # - send_out_msg() to send an OUT message
    # - get_in_msg()   to request an IN message
    def send_msg(self, msg):
        # print('------')
        # print(self.make_msg())
        self.socket.send(msg)

    # read a message with payload of size `payload_len` from the drone
    # used by:
    # - send_out_msg() to read the empty acknowledgement packet
    # - get_in_msg()   to read the response to an IN message request
    def get_msg(self, payload_len):
        # 2 byte header
        # 1 byte dir
        # 1 byte cmd
        # 1 byte length
        # length bytes payload
        # 1 byte crc
        msg = self.socket.recv(6 + payload_len, socket.MSG_WAITALL)
        return parse_out(msg)

    # request a IN message from the drone, and read the response
    # used by:
    # - get_raw_vals() to request the raw sensor values
    def get_in_msg(self, cmd, payload_len):
        msg = make_in(cmd, b"")
        self.send_msg(msg)
        _, payload = self.get_msg(payload_len)
        return payload

    # ------------------- Controller -------------------

    # the controller support is part of the Command class, but using it is optional
    # drone can be controlled without a controller too (like in task 2)

    # listeners are set up for each button, and the axes are read by polling

    # attach a controller to the object
    def add_controller(self, controller):
        self.controller = controller
        for button in controller.buttons:
            button.when_pressed = lambda btn : (
                self.button_handler(btn)
            )
        # axes are read by polling, not by events (see self.get_controller_axes)

    def button_handler(self, button):
        if button.name == "button_a":
            self.disarm()
        elif button.name == "button_b":
            self.arm()
        elif button.name == "button_x":
            self.takeoff()
        elif button.name == "button_y":
            self.land()
        elif button.name == "button_mode":
            self.calib()
        else:
            print("unused button pressed: " + button.name)

    # read controller axes and update values
    def read_controller_axes(self):
        # moving joystick up makes y negative, so -ve sign for throttle and pitch
        self.throttle = 1500 + int(self.controller.axis_l.y * -200)
        self.yaw = 1500 + int(self.controller.axis_l.x * 300)
        self.pitch = 1500 + int(self.controller.axis_r.y * -200)
        self.roll = 1500 + int(self.controller.axis_r.x * 200)

if __name__ == "__main__":
    # simple test, using the controller to control the drone

    from xbox360controller import Xbox360Controller
    import itertools
    with Xbox360Controller(axis_threshold=0, raw_mode=0) as controller:
        command = Command("192.168.4.1", interface='wlp170s0')
        command.add_controller(controller)

        command.disarm()
        # command.calib()


        try:
            i=0
            for i in itertools.cycle(range(10)):
                time.sleep(0.1)
                if i == 0:
                    print(command.get_raw_vals())
                command.send() # reads controller axes before sending
        except KeyboardInterrupt:
            try:
                command.land()
            except KeyboardInterrupt:
                command.disarm()
