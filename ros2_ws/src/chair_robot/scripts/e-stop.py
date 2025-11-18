#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from geometry_msgs import Twist

from inputs import get_gamepad

class EStop(Node):
    def __init__(self):
        super().__init__("e-stop")
        self.stop_pub = self.publisher(Twist, "/cmd_vel_stop")
        self.timer = self.create_timer(.1, self.run_loop)

    def run_loop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        while True:
            events = get_gamepad()
            for event in events:
                if event.code == BTN_EAST:
                    while True:
                        self.stop_pub.publish(msg)
                        print("E-STOP PRESSED")

if __name__ == "main":
    rclpy.init()
    rclpy.spin(EStop())
    rclpy.shutdown()
