#!/usr/bin/env python3
import math
import os
import yaml
from time import sleep
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, TwistStamped, Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Bool
from transform_helper import FrameUpdater
from pose_from_aruco import VideoProcess
from geometry_msgs.msg import Twist
from gpiozero import PWMLED, LED

class RobotState(Node):
    def __init__(self, use_lidar=False, follow_id=-1):
        super().__init__("robot_state",namespace="robot0")
        self.timer = self.create_timer(0.01,self.main_loop)
        self.name = "robot0"
        self.ns
        self.id = 0
        self.follow_id = follow_id # id of robot; person target has id -1 
        self.has_lidar = use_lidar
        self.teleop_sub = self.create_subscription(Int32,"/use_teleop",self.use_teleop_callback,10)
        self.state = "follow"

        # Create TF2 frame broadcasters
        self.robot_to_world = FrameUpdater(node=self,parent=self.name,child="world",id=self.id)

        # If we're the base robot, do the target frame update
        if self.id == 0:
            self.target_to_world = FrameUpdater(node=self,parent="target",child="world",id=-1)
        
        # Create video processing node
        self.vid_process = VideoProcess(node=self,use_gui=False,channel=0,follow_target=-1,name=self.name)

        # Set pins for motor control
        self.motor1 = PWMLED(18) #right one (for now)
        self.motor2 = PWMLED(27) #lt one (for now)

        self.m1high = LED(17)
        self.m1low = LED(22)
        self.m2high = LED(23)
        self.m2low = LED(24)

        self.pose_sub = self.create_subscription(Float32MultiArray, "/marker_pose", self.process_pose, 10)
        self.cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self.process_twist,10)

        self.heartbeat_sub = self.create_subscription(Bool,"/heartbeat",self.heartbeat_callback,10)
        self.latest_hb_time = self.get_clock().now().nanoseconds

        self.pose_bound = 2 # distance epsilon to pass before we assume the target has moved
        self.teleop_state = 0 # id of robot controlled by teleop
        self.reorient_flag = True

    def main_loop(self):        
        match (self.state):
            case "follow":
                pose_updates = self.vid_process.process_frame()
                for update in pose_updates:
                    camera_frame = self.name + "/camera"
                self.state_change()
            case "hold":
                pose_updates = self.vid_process.process_frame()
                self.state_change()
            case "teleop":
                pass
                self.state_change()
            case "stop":
                self.drive_raw(0,0)
                self.state_change()
                return
            case "search":
                pose_updates = self.vid_process.process_frame()
                self.state_change()
            case "waypoint":
                pass
                self.state_change
    
    def state_change(self):
        match (self.state):
            case "follow":
                # If we're in follow, flip state when we get close
                pass
            case "hold":
                # If we're in hold, wait for target to go away
                pass
            case "teleop":
                # If we're in teleop, rely on controller to change state
                pass
            case "stop":
                # If we're stopped, resume is only done async, not here
                pass 
            case "search":
                # Resume if we find target
                pass
            case "waypoint":
                # Hold if we reach waypoint
                pass
    
    def drive_raw(self, m1, m2):
        print(f"Driving! {m1}, {m2}")
        max_percent = 0.8
        m1 = min(max(-max_percent,m1),max_percent)
        m2 = min(max(-max_percent,m2),max_percent)
        print(f"Filtered m1 and m2: {m1}, {m2}")
        if m1 < 0:
            self.m1low.on()
            self.m1high.off()
            self.motor1.value = -1*m1
            print(f"M1 low on, M1 high off, value {-1*m1}")
        else: 
            self.m1low.off()
            self.m1high.on()
            self.motor1.value = m1
            print(f"M1 low off, M1 high on, value {m1}")

        if m2 < 0:
            self.m2low.on()
            self.m2high.off()
            self.motor2.value = -1*m2
            print(f"M2 low on, M2 high off, value {-1*m2}")
        else: 
            self.m2low.off()
            self.m2high.on()
            self.motor2.value = m2
            print(f"M2 low on, M2 high off, value {m1}")

    def process_twist(self, msg: Twist):
        # Change for actual units / something parameterizable
        print(f"I got a twist! {msg.angular.z} angular, {msg.linear.x} linear")
        ang = msg.angular.z
        lin = msg.linear.x
        if not (self.state == "stop" or self.state == "drive"):
            self.drive_raw(lin + ang, lin - ang)

    def heartbeat_callback(self, msg: Bool):
        self.latest_hb_time = self.get_clock().now().nanoseconds

    def use_teleop_callback(self,msg):
        # Update state --- -1 is auto, teleop if equal to my id, stop if not
        val = msg.data
        self.teleop_state = val
        print(f"Received val: {val}")
        if val == self.id:
            self.state = "teleop"
        elif val == -1:
            self.state = "follow"
        else:
            self.state = "stop"
    
    def publish_movement_cmd(self,linear,angular):
        cmd = TwistStamped()
        cmd.header.frame_id = self.name
        cmd.header.stamp.nanosec = self.get_clock().now().nanoseconds

        lin = Vector3(linear)
        ang = Vector3(angular)
        cmd.twist.linear = lin
        cmd.twist.angular = ang

        self.cmd_vel_pub.publish(cmd)

                   
if __name__ == "__main__":
    rclpy.init()
    robot = RobotState()
    rclpy.spin(robot)
    rclpy.shutdown()
