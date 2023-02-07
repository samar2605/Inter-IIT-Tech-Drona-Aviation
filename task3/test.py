import pygame
import sys
import time
from multinetwork import Command

pygame.init()
display = pygame.display.set_mode((300, 300))

# laptop builtin card
command1 = Command("192.168.4.1", "wlp170s0")

# usb wifi card
command2 = Command("192.168.4.1", "wlp0s20f0u4")

command1.disarm()
command2.disarm()

command1_state = False
command2_state = False
while True:
    time.sleep(0.1)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                command1_state = True
                print("w")
            elif event.key == pygame.K_s:
                command1_state = False
                print("s")
            elif event.key == pygame.K_UP:
                command2_state = True
                print("UP")
            elif event.key == pygame.K_DOWN:
                command2_state = False
                print("DOWN")

    if command1_state:
        command1.arm()
    else:
        command1.disarm()

    # if command2_state:
        # command2.boxarm()
    # else:
        # command2.disarm()
print("close2")
