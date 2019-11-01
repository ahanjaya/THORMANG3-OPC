#!/usr/bin/env python3
import pygame
import rospy
import rospkg
import numpy as np

rospy.init_node('pioneer_deep_cross_arm')

pygame.init()
screen     = pygame.display.set_mode((640, 480))
font      = pygame.font.SysFont("comicsansms", 72)
main_rate = rospy.Rate(10)
count = 0

while not rospy.is_shutdown():
    count += 1

    if count > 20:
        # text = font.render("right arm on bottom", True, (0, 128, 0))
        # text = font.render("", True, (0, 128, 0))
        # screen.fill((0, 0, 0))
        pygame.quit()
    else:
        text = font.render("left arm on bottom", True, (0, 0, 128))
        screen.fill((255, 255, 255))

    screen.blit(text, (320 - text.get_width() // 2, 240 - text.get_height() // 2))
    pygame.display.flip()


    main_rate.sleep()