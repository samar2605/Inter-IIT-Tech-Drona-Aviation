from control_class import *
import time
command = Command("192.168.4.1")
try:
    for i in range(10):
        command.disarm()
        time.sleep(0.1)
#     start = time.time()
    command.takeoff()
#     end = time.time()

    #print('takeoff time:', end - start)
    #time.sleep(1)
    for i in range(20):
        command.roll =  1512
        command.pitch = 1515
        command.throttle = 1470
        command.send()
        print('sending')
        time.sleep(0.05)
    for i in range(55):
        command.roll =  1518
        command.pitch = 1560
        command.throttle = 1470
        command.send()
        print('forward')
        time.sleep(0.05)

    for i in range(30):
        command.roll =  1460
        command.pitch = 1480
        command.throttle = 1470
        command.send()
        print('left')
        time.sleep(0.05)

    for i in range(50):
        command.roll =  1518
        command.pitch = 1435
        command.throttle = 1470
        command.send()
        print('back')
        time.sleep(0.05)

#     start = time.time()
#     print('landing')
    command.land()
#     end = time.time()
#     print('land')
#     print('time = ',end - start)
except KeyboardInterrupt:
    
    command.land()
    command.__del__()