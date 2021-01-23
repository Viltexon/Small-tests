#! /usr/bin/env python

import rospy
import pygame

from geometry_msgs.msg import Point


def start():
    rospy.init_node('controls')

    pygame.init()
    screen_dim = (200, 200)
    screen = pygame.display.set_mode(screen_dim)
    pygame.display.set_caption("controls")

    direction_pub = rospy.Publisher('direction', Point, queue_size=1)

    rate = rospy.Rate(5)

    running = True
    while not rospy.is_shutdown() and running:

        direction = Point(0, 0, 0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        arrow_color = (0, 0, 0)
        keystate = pygame.key.get_pressed()
        if keystate[pygame.K_LEFT]:
            direction = Point(-1, 0, 0)
            pygame.draw.polygon(screen, arrow_color,
                                [[screen_dim[0]//8, screen_dim[1]//2], [screen_dim[0]//4, screen_dim[1]//4], [screen_dim[0]//4, screen_dim[1]*3//4]], 5)
        if keystate[pygame.K_RIGHT]:
            direction = Point(1, 0, 0)
            pygame.draw.polygon(screen, arrow_color,
                                [[screen_dim[0]*7//8, screen_dim[1]//2], [screen_dim[0]*3//4, screen_dim[1]//4], [screen_dim[0]*3//4, screen_dim[1]*3//4]], 5)
        if keystate[pygame.K_UP]:
            direction = Point(0, -1, 0)
            pygame.draw.polygon(screen, arrow_color,
                                [[screen_dim[0]//2, screen_dim[1]//8], [screen_dim[0]//4, screen_dim[1]//4], [screen_dim[0]*3//4, screen_dim[1]//4]], 5)
        if keystate[pygame.K_DOWN]:
            direction = Point(0, 1, 0)
            pygame.draw.polygon(screen, arrow_color,
                                [[screen_dim[0]//2, screen_dim[1]*7//8], [screen_dim[0]//4, screen_dim[1]*3//4], [screen_dim[0]*3//4, screen_dim[1]*3//4]], 5)
        pygame.display.flip()

        direction_pub.publish(direction)
        rate.sleep()


if __name__ == "__main__":
    start()
