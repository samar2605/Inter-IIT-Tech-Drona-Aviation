from control_class import Command
import time

command = Command("192.168.4.1")


for i in range(10):
    command.disarm()
    time.sleep(0.1)

command.land()