#!/usr/bin/env python3
import math
import os
import sys
import yaml
from time import sleep
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, TwistStamped, Vector3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32, Float32MultiArray, Bool
from transform_helper import FrameUpdater
from pose_from_aruco import VideoProcess
from geometry_msgs.msg import Twist
from gpiozero import PWMLED, LED

class RobotState(Node):
    def __init__(self, use_lidar=False, follow_id=-1, channel=0):
        super().__init__("robot_state",namespace="robot0")
        self.timer = self.create_timer(0.01,self.main_loop)
        self.name = "robot0"
        self.id = 0
        self.follow_id = follow_id # id of robot; person target has id -1 
        self.has_lidar = use_lidar
        self.teleop_sub = self.create_subscription(Int32,"/use_teleop",self.use_teleop_callback,10)
        self.state = "follow"

        # Are we the leader or follower robot?
        self.declare_parameter("is_trashcan",True)
        self.is_trashcan = self.get_parameter("is_trashcan").get_parameter_value().bool_value

        if self.is_trashcan:
            self.leader_reached_target_pub = self.create_publisher(Bool,"/leader_reached_target",10)
            self.waypoint_pub = self.create_publisher(TransformStamped,"/waypoint",10)
            # publisher for laser scan?
        else:
            self.leader_reached_target_sub = self.create_subscription(Bool,"/leader_reached_target",self.leader_reached_target_callback,10)
            self.waypoint_sub = self.create_subscription(TransformStamped,"/waypoint",self.waypoint_callback,10)
            # sub for laser scan?

        # Create TF2 frame broadcasters
        self.robot_to_world = FrameUpdater(node=self,follow_target=-1,parent=self.name,child="world",id=self.id)

        # We need to continually use this FrameUpdater to get our distance from the robots/target
        self.frame_update_timer = self.create_timer(0.01,self.robot_to_world.read_transforms)

        # If we're the base robot, do the target frame update
        if self.id == 0:
            self.target_to_world = FrameUpdater(node=self,follow_target=-1,parent="target",child="world",id=-1)
        
        # Create video processing node
        self.vid_process = VideoProcess(node=self,use_gui=False,channel=channel,follow_target=-1,name=self.name)

        # Set pins for motor control
        self.motor1 = PWMLED(20) #right one (for now)
        self.motor2 = PWMLED(21) #lt one (for now)

        self.m1high = LED(17)
        self.m1low = LED(22)
        self.m2high = LED(23)
        self.m2low = LED(24)

        if not self.is_trashcan:
            self.motor1 = PWMLED(2)
            self.m1high = LED(3)
            self.m1low = LED(4)
            self.motor2 = PWMLED(17)
            self.m2high = LED(27)
            self.m2low = LED(22)

        #self.pose_sub = self.create_subscription(Float32MultiArray, "marker_pose", self.process_pose, 10)
        self.cmd_vel_sub = self.create_subscription(Twist,"cmd_vel",self.process_twist,10)
        self.cmd_vel_pub = self.create_publisher(Twist,"cmd_vel",10)

        self.heartbeat_sub = self.create_subscription(Bool,"/heartbeat",self.heartbeat_callback,10)
        self.latest_hb_time = None # should be (seconds, ns)

        self.pose_bound = 0.2 # distance epsilon to pass before we assume the target has moved
        self.follow_distance = 0.8 # distance to maintain following
        self.teleop_state = 0 # id of robot controlled by teleop
        self.reorient_flag = True

        self.kP_angle = 0.1
        self.kI_angle = 0.0
        self.kD_angle = 0.0
        self.prev_err_angle = 0.0
        self.err_list_angle = []

        self.kP_lin = 0.1
        self.kI_lin = 0.0
        self.kD_lin = 0.0
        self.prev_err_lin = 0.0
        self.err_list_lin = []

        self.last_PID_times = self.get_clock().now().to_msg()

    def leader_reached_target_callback(self, msg: Bool):
        """
        Invoked when the leader robot reaches the target (flips state to Hold)
        """
        if msg.data:
            # The leader reached the target --- we should now follow a waypoint
            self.state = "waypoint"
        else:
            # The leader lost the target --- we should keep following it or holding position
            target_distance = self.robot_to_world.distances[self.follow_id]
            if target_distance > self.follow_distance + self.pose_bound:
                self.state = "follow"
            else:
                self.state = "hold"

    def waypoint_callback(self, msg: TransformStamped):
        """
        Invoked when the leader robot reaches the target and passes a waypoint for the follower
        """

    def main_loop(self):      
        print(f"\nbeginning main loop in state {self.state}\n")  
        match (self.state):
            case "follow":
                pose_updates = self.vid_process.process_frame()
                if pose_updates is not None and len(pose_updates) > 0:
                    for update in pose_updates:
                        camera_frame = self.name + "/camera"
                        print(f"Got pose update!")
                try:
                    target_pos_tf = self.robot_to_world.tf_buffer.lookup_transform(
                        self.robot_to_world.robot_names[self.follow_id],
                        self.name,
                        rclpy.time.Time()
                    )
                except:
                    self.state_change()
                    return
                target_pos = target_pos_tf.transform.translation
                print(f"Driving to transform: {target_pos}")
                self.drive_to_transform(target_pos)
                self.state_change()
            case "hold":
                pose_updates = self.vid_process.process_frame()
                self.drive_raw(0.0,0.0)
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
                # We want to navigate to the waypoint (world frame) from where we are now.
                if self.is_trashcan:
                    pass
                self.state_change()
    
    def state_change(self):
        # How far are we from the target?
        if not self.follow_id in self.robot_to_world.distances.keys():
            print("Don't know where target is!")
            target_distance = None
        else:
            target_distance = self.robot_to_world.distances[self.follow_id]
        print(f"target distance: {target_distance}\n")
        match (self.state):
            case "follow":
                # If we're in follow, flip state when we get close
                if target_distance is None:
                    self.state = "search"
                    return
                if target_distance < self.follow_distance:
                    self.state = "hold"
            case "hold":
                # If we're in hold, wait for target to go away
                if target_distance is None:
                    self.state = "search"
                    return
                if target_distance > self.follow_distance + self.pose_bound or target_distance < self.follow_distance - self.pose_bound:
                    self.state = "follow"
            case "teleop":
                # If we're in teleop, rely on controller to change state
                pass
            case "stop":
                # If we're stopped, resume is only done async, not here
                pass 
            case "search":
                # Resume if we find target
                self.drive_raw(0,0)
                if target_distance is not None:
                    self.state = "follow"
            case "waypoint":
                # Hold if we reach waypoint
                pass
    
    def drive_to_transform(self, translation: Vector3):
        """
        Attempts to drive towards a target, given the translation to it
        from the robot's center.
        """
        # If we're too close, stop
        # Y is our unused dimension (vertical) as we move in a plane
        last_time = self.last_PID_times
        current_time = self.get_clock().now().to_msg()
        self.last_PID_times = current_time
        duration = Duration(seconds=current_time.sec - last_time.sec, nanoseconds=current_time.nanosec - last_time.nanosec)
        dt = duration.nanoseconds * 1e-9
        print(f"dt: {dt}")

        if translation.x**2 + translation.z**2 < self.follow_distance**2:
            #self.drive_raw(0,0)
            print("too close! correcting")
        print(f"Getting: {translation.x}, {translation.z}")

        angle_err = np.arctan2(translation.x,translation.z)
        linear_err = np.sqrt(translation.x**2 + translation.z**2) - self.follow_distance
        err_queue_size = 20

        self.err_list_angle.append(angle_err)
        if len(self.err_list_angle) > err_queue_size:
            self.err_list_angle.pop(0)

        self.err_list_lin.append(linear_err)
        if len(self.err_list_lin) > err_queue_size:
            self.err_list_lin.pop(0)

        # PID control of angle
        angle_err_I = sum(self.err_list_angle) * dt
        angle_err_D = (angle_err - self.prev_err_angle) / dt

        angle_corr = self.kP_angle * angle_err + self.kI_angle * angle_err_I + self.kD_angle * angle_err_D
        cmd = Twist()
        cmd.angular.y = angle_corr

        # PID control of angle
        lin_err_I = sum(self.err_list_lin) * dt
        lin_err_D = (linear_err - self.prev_err_lin) / dt

        lin_corr = self.kP_lin * linear_err + self.kI_lin * lin_err_I + self.kD_lin * lin_err_D
        cmd.linear.z = lin_corr

        print(f"Publishing cmd: {cmd}\nIn response to angular error {angle_err} and linear error {linear_err}")
        self.cmd_vel_pub.publish(cmd)
    
    def drive_raw(self, m1, m2):
        # Don't drive if we don't have a heartbeat
        if self.latest_hb_time is None:
            m1 = 0.0
            m2 = 0.0
        else:
            hb_time = self.latest_hb_time[0] + float(self.latest_hb_time[1]) * 1e-9
            now = self.get_clock().now().seconds_nanoseconds()
            now_time = now[0] + float(now[1]) * 1e-9
            time_since_hb = now_time - hb_time
            #print(f"Time since heartbeat: {time_since_hb}")
            if time_since_hb > 1.0:
                m1 = 0.0
                m2 = 0.0

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
        print(f"I got a twist! {msg.angular.y} angular, {msg.linear.z} linear")
        ang = msg.angular.y
        lin = msg.linear.z
        if not (self.state == "stop" or self.state == "drive"):
            self.drive_raw(lin + ang, lin - ang)

    def heartbeat_callback(self, msg: Bool):
        self.latest_hb_time = self.get_clock().now().seconds_nanoseconds()

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

                   
if __name__ == "__main__":
    if len(sys.argv) < 2:
        channel = 4
    else:
        channel = int(sys.argv[1])
    rclpy.init()
    robot = RobotState(channel=channel)
    rclpy.spin(robot)
    rclpy.shutdown()
