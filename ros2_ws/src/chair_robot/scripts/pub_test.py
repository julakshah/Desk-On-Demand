#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class PubTest(Node):
    def __init__(self):
        super().__init__("Simple_Publisher")
        self.pub = self.create_publisher(String,"/sample_topic",10)
        self.timer = self.create_timer(1.0,self.timer_callback)
        self.seconds = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello! I've been active for {self.seconds} seconds"
        self.seconds += 1
        self.pub.publish(msg)
        print(f"Publishing {msg.data}")

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(PubTest())
    rclpy.shutdown()
