#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""

from cmath import cos, sin
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Twist
from time import *
import numpy as np
from getch import getch, pause
import sys
import termios
import tty
import keyboard


class KeyBoardControl(Node):

    def __init__(self):

        super().__init__('drone_image_publisher')
        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 2)
        self.move = Twist()
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.movetello)

        self.msg = 'No message'
        self.moveBindings = {
        'i': (1, 0, 0, 0),
        'o': (1, 0, 0, -1),
        'j': (0, 0, 0, 1),
        'l': (0, 0, 0, -1),
        'u': (1, 0, 0, 1),
        ',': (-1, 0, 0, 0),
        '.': (-1, 0, 0, 1),
        'm': (-1, 0, 0, -1),
        'O': (1, -1, 0, 0),
        'I': (1, 0, 0, 0),
        'J': (0, 1, 0, 0),
        'L': (0, -1, 0, 0),
        'U': (1, 1, 0, 0),
        '<': (-1, 0, 0, 0),
        '>': (-1, -1, 0, 0),
        'M': (-1, 1, 0, 0),
        't': (0, 0, 1, 0),
        'b': (0, 0, -1, 0),
        }

        self.speedBindings = {
        'q': (1.1, 1.1),
        'z': (.9, .9),
        'w': (1.1, 1),
        'x': (.9, 1),
        'e': (1, 1.1),
        'c': (1, .9),
        }

        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0


    def getKey(self,settings):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def saveTerminalSettings(self):
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)


    def restoreTerminalSettings(self,old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


    def vels(self,speed, turn):
        return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

    def movetello(self):

        settings = self.saveTerminalSettings()
        key = self.getKey(settings)
        if key in self.moveBindings.keys():
            self.x = self.moveBindings[key][0]
            self.y = self.moveBindings[key][1]
            self.z = self.moveBindings[key][2]
            self.th = self.moveBindings[key][3]
        elif key in self.speedBindings.keys():
            self.speed = self.speed * self.speedBindings[key][0]
            self.turn = self.turn * self.speedBindings[key][1]

            print(self.vels(self.speed, self.turn))
            if (self.status == 14):
                print(self.msg)
            self.status = (self.status + 1) % 15
        else:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0

        #twist = msg.Twist()
        self.move.linear.x = self.x * self.speed
        self.move.linear.y = self.y * self.speed
        self.move.linear.z = self.z * self.speed
        self.move.angular.x = 0.0
        self.move.angular.y = 0.0
        self.move.angular.z = self.th * self.turn
        self.publisher_.publish(self.move)
    
def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    key_publisher = KeyBoardControl()

    # Spin the node so the callback function is called.
    rclpy.spin(key_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':

    main()