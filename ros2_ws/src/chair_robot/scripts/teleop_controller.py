#!/usr/bin/env python
"""
ROS2 Node to add teleop control to an autonomous robot
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int32, Bool
from rclpy.qos import qos_profile_sensor_data
import termios
import tty
import sys

class TeleopController(Node):
    """
    Node for passing velocity commands pased on teleop control
    to robot. Pauses function of autonomous node when actively 
    publishing to /cmd_vel.

    Attributes: 
        teleop_pub: Publisher to /use_teleop; Int32 specifies which robot is being controlled
        vel_pub: Publisher to /cmd_vel topic for robot
        is_publishing (bool): internal state, specifies robot in 
            teleop mode
    """
    settings = termios.tcgetattr(sys.stdin)
    key = None

    def __init__(self):
        super().__init__('teleop_node')
        self.teleop_pub = self.create_publisher(Int32, 'use_teleop', 10)
        self.vel_pub  = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.01,self.run_loop)
        self.is_publishing = True
        self.id_controlled = -1
        self.speed_percent = 0.5
        self.linear = 0.0
        self.angular = 0.0
        self.hb_pub = self.create_publisher(Bool, "heartbeat", 10)

    def run_loop(self):
        """
        Loop to run teleop control code for the node
        Listens to keypresses in stdin and passes velocity commands
        if WASD are pressed.
        Space represents stop, and enter hands control back to 
        the autonomous node. 
        Exits on SIGINT / Ctrl + C
        """
        # Publish heartbeat
        self.hb_pub.publish(Bool(True))

        key = self.get_key()
        print(f"key = {key}")

        # Perform logic based on key identifier
        if key == ' ':
            print("Swapping to manual control")
            msg = Int32()
            msg.data = True
            self.teleop_pub.publish(msg)
            self.is_publishing = True
            self.drive()

        if key == '\n' or key == '\r':
            print("Swapping to automatic control")
            msg = Int32()
            msg.data = -1
            self.teleop_pub.publish(msg)
            self.is_publishing = False

        if key in {'0','1','2','3','4','5','6','7','8','9'}:
            print(f"Selected robot number {key} to control")
            msg = Int32()
            msg.data = int(key)
            self.teleop_pub.publish(msg)

        # Only give velocity commands if currently set to publish
        # Otherwise, would interfere with autonomous control
        if self.is_publishing:
            if key == 'w':
                print("Moving forward")
                self.linear = 0.4
                self.angular = 0.0
                self.drive(linear=self.linear*self.speed_percent,angular=self.angular*self.speed_percent)
            if key == 'a':
                print("Turning left")
                self.linear = 0.0
                self.angular = 0.4
                self.drive(linear=self.linear*self.speed_percent,angular=self.angular*self.speed_percent)
            if key == 's':
                print("Moving backward")
                self.linear = -0.4
                self.angular = -0.0
                self.drive(linear=self.linear*self.speed_percent,angular=self.angular*self.speed_percent)
            if key == 'd':
                print("Turning right")
                self.linear = 0.0
                self.angular = -0.4
                self.drive(linear=self.linear*self.speed_percent,angular=self.angular*self.speed_percent)
        
        if key == 'j':
            self.speed_percent += 0.05
            self.speed_percent = min(1,max(0,self.speed_percent))
            print(f"Increment speed to {self.speed_percent} of max")
            self.drive(linear=self.linear*self.speed_percent,angular=self.angular*self.speed_percent)

        if key == 'k':
            self.speed_percent -= 0.05
            self.speed_percent = min(1,max(0,self.speed_percent))
            print(f"Decrement speed to {self.speed_percent} of max")
            self.drive(linear=self.linear*self.speed_percent,angular=self.angular*self.speed_percent)


        # Stop program when Ctrl C pressed
        if key == '\x03':
            raise KeyboardInterrupt
        

    def drive(self,linear=0.0,angular=0.0):
        """
        Publish velocity command to /cmd_vel
        Controls robot movement

        Args:
            linear (float): linear velocity in the x direction to
            publish to /cmd_vel, in m/s
            angular (float): angular velocity about z axis to
            publish to /cmd_vel, in rad/s
        """

        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular

        self.vel_pub.publish(twist)

    def get_key(self):
        """
        Reads a keypress from the active terminal window
        
        Returns:
            key identifier for the next key entered in stdin
        """
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def __del__(self):
        # Shutdown by publishing a message to init control (stopping robots)
        msg_int = Int32()
        msg_int.data = 999
        self.drive(0.0,0.0)
        self.teleop_pub.publish(msg_int)

def main():
    rclpy.init()
    node = TeleopController()
    rclpy.spin(node)
    node.drive(linear=0.0,angular=0.0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
