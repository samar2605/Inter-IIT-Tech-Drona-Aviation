from control_class import *

command = Command("192.168.4.1")

for i in range(10):
    command.disarm()
    time.sleep(0.1)

# for i in range(10):
#     command.arm()
#     time.sleep(0.1)
#command.takeoff()
# # for i in range(20):
# #     command.throttle =  1550
# #     time.sleep(0.1)
# # for i in range(10):
# #     command.arm()
# #     time.sleep(0.1)
# #command.takeoff()
# # for i in range(5):
# #     command.boxarm()
# #     print('boxarm')
# #     time.sleep(0.1)
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

command.land()
