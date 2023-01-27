from control_class import *

command = Command("192.168.4.1")
try:
    for i in range(10):
        command.disarm()
        time.sleep(0.1)

    # for i in range(10):
    #     command.arm()
    #     time.sleep(0.1)
    command.takeoff()
    # for i in range(10):
    #     command.boxarm()
    #     time.sleep(0.1)

    print('takeoff')
    time.sleep(1)
    for i in range(50):
        command.roll =  1530
        command.pitch = 1475
        command.throttle = 1475
        command.send()
        print('sending')
        time.sleep(0.1)


    # # for i in range(10):
    # #     command.arm()
    # #     time.sleep(0.1)
    # #command.takeoff()
    # for i in range(30):
    #     command.boxarm()
    #     print('boxarm')
    #     time.sleep(0.1)
    # command.land()
    # t = 1900
    # for i in range(60):
    #     command.throttle = t
    #     t = t- 5
    #     print('throttle = ',t)
    #     command.send()
    #     time.sleep(0.1)





    # # for i in range(10):
    # #     command.pitch = 1400
    # #     command.send()
    # #     time.sleep(0.1)

    # # for i in range(10):
    # #     command.pitch = 1500
    # #     command.send()
    # #     time.sleep()
    start = time.time()
    print('landing')
    command.land()
    end = time.time()
    print('land')
    print('time = ',end - start)
except KeyboardInterrupt:
    command.land()