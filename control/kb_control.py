import pygame
import sys
import time
from multinetwork import Command

pygame.init()
display = pygame.display.set_mode((300, 300))

# laptop builtin card
command = Command("192.168.4.1")

command.disarm()

while True:
    time.sleep(0.1)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            command.land()
            time.sleep(3)
            command.disarm()
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                command.throttle = 1800
            elif event.key == pygame.K_a:
                command.yaw = 1400
            elif event.key == pygame.K_s:
                command.throttle = 1300
            elif event.key == pygame.K_d:
                command.yaw = 1600

            elif event.key == pygame.K_UP:
                command.pitch = 1600
            elif event.key == pygame.K_DOWN:
                command.pitch = 1400
            elif event.key == pygame.K_LEFT:
                command.roll = 1400
            elif event.key == pygame.K_RIGHT:
                command.roll = 1600

            elif event.key == pygame.K_t:
                command.takeoff()
            elif event.key == pygame.K_l:
                command.land()
            elif event.key == pygame.K_e:
                command.arm()
            elif event.key == pygame.K_x:
                command.disarm()

            elif event.key == pygame.K_q:
                command.disarm()
                pygame.quit()
                sys.exit()

        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_w or event.key == pygame.K_s:
                command.throttle = 1550
            elif event.key == pygame.K_a or event.key == pygame.K_d:
                command.yaw = 1500
            elif event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                command.pitch = 1500
            elif event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                command.roll = 1500

    command.send()
