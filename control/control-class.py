import struct
import socket
import time

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

def msp_set_command(command):
    payload = struct.pack("<H", command)
    return make_in(0xd9, payload)

ACC_CALIB = make_in(0xcd, b"")

MAG_CALIB = make_in(0xce, b"")

class Command:
    def __init__(self, ip_addr):
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sender.connect((ip_addr, 23))

        self.throttle = 1500
        self.yaw = 1500
        self.pitch = 1500
        self.roll = 1500

        # TODO: is 1500 for aux1/aux2 fine?
        self.aux1 = 2100
        self.aux2 = 900
        self.aux3 = 1500
        self.aux4 = 900

        self.lastButton = None
        self.controller = None

        # if zero, command is of type MSP_SET_RAW_RC
        # if non-zero, self.cmd is the payload for MSP_SET_COMMAND
        # 1 for takeoff, 2 for land
        self.cmd = 0

    def __del__(self):
        self.sender.close()

    # Xbox 360 controller support
    def add_controller(self, controller):
        self.controller = controller
        # for axis in controller.axes:
            # axis.when_moved = lambda axis : (
                # self.axis_handler(axis)
            # )
        for button in controller.buttons:
            button.when_pressed = lambda btn : (
                self.button_handler(btn)
            )

    def is_armed(self):
        return 1300 < self.aux4 < 1700

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

    def set_raw_vals(self, throttle=1500, yaw=1500, pitch=1500, roll=1500):
        self.throttle = throttle
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.send()

    def takeoff(self):
        if self.is_armed():
            # already armed => don't try to takeoff (might've already taken off)
            return

        self.disarm()
        time.sleep(0.1)

        self.boxarm()
        self.cmd = 1
        self.send()

        # time.sleep(5)
        self.boxarm()

    def land(self):
        self.cmd = 2
        self.send()
        time.sleep(5)
        self.disarm()

    def calib(self):
        if self.is_armed():
            return
        self.send_msg(ACC_CALIB)
        self.send_msg(MAG_CALIB)

    def make_msg(self):
        if self.cmd == 0:
            return msp_set_raw_rc(self.roll, self.pitch, self.throttle, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4)
        else:
            return msp_set_command(self.cmd)

    def button_handler(self, button):
        if button.name == "button_a":
            self.disarm()
        elif button.name == "button_b":
            self.arm()
        elif button.name == "button_x":
            self.takeoff()
        elif button.name == "button_y":
            self.land()

    def axis_handler(self, axis):
        # x and y are in [-1.0, 1.0]
        # -axis.y because moving joystick up is negative and down is positive, and we don't want that
        x, y = axis.x, -axis.y

        if axis.name == "axis_l":
            self.yaw = 1500 + int(x * 200)
            self.throttle = 1550 + int(y * (200 if y > 0 else 100))
        else:
            # axis.name == "axis_r"
            self.roll = 1500 + int(x * 200)
            self.pitch = 1500 + int(y * 200)

    def get_controller_axes(self):
        self.throttle = 1500 + int(self.controller.axis_l.y * -200)
        self.yaw = 1500 + int(self.controller.axis_l.x * 300)
        self.pitch = 1500 + int(self.controller.axis_r.y * -200) # moving joystick up makes y negative
        self.roll = 1500 + int(self.controller.axis_r.x * 200)

    def send(self):
        if self.is_armed() and self.controller is not None:
            self.get_controller_axes()
        self.send_msg(self.make_msg())

    def send_msg(self, msg):
        print('------')
        print(self.make_msg())
        self.sender.send(msg)

if __name__ == "__main__":
    from xbox360controller import Xbox360Controller
    with Xbox360Controller(axis_threshold=0, raw_mode=0) as controller:
        command = Command("192.168.4.1")
        command.add_controller(controller)

        command.disarm()
        command.calib()

        try:
            while True:
                time.sleep(0.1)
                command.send()
        except KeyboardInterrupt:
            try:
                command.land()
            except KeyboardInterrupt:
                command.disarm()
