from control_class import Command
import time

command = Command("192.168.4.1")

for i in range(10):
    command.disarm()
    time.sleep(0.1)

command.takeoff()
for i in range(10):
    command.boxarm()
    time.sleep(0.1)

for i in range(50):
    command.set_attitude(throttle=1500,yaw=1500,pitch = 1500,roll = 1500)
    time.sleep(0.1)

for i in range(10):
    command.boxarm()
    time.sleep(0.1)

command.land()