#!/usr/bin/env python3
# ROS2 nodes
import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

# Pi gpio control
<<<<<<< HEAD
from gpiozero import PWMLED
from time import sleep, time
=======
from gpiozero import PWMLED, LED
from time import sleep
>>>>>>> devel-robot-base
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool


class TestMotorSpin(Node):
    def __init__(self):
        super().__init__("movement")

        self.motor1 = PWMLED(18) #right one (for now)
        self.motor2 = PWMLED(27) #lt one (for now)

        self.m1high = LED(17)
        self.m1low = LED(22)
        self.m2high = LED(23)
        self.m2low = LED(24)

        self.timer = self.create_timer(0.1, self.run_loop)

        self.new_target = None
        self.current_target = None
        self.pose_sub = self.create_subscription(Float32MultiArray, "/marker_pose", self.process_pose, 10)
        self.cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self.process_twist,10)
        self.use_teleop_sub = self.create_subscription(Int32,"/use_teleop",self.process_teleop,10)

        self.heartbeat_sub = self.create_subscription(Bool,"/heartbeat",self.heartbeat_callback,10)
        self.latest_hb_time = self.get_clock().now().nanoseconds

        self.pose_bound = 2
        self.id = 0 # id of robot
        self.teleop_state = 0 # id of robot controlled by teleop
        self.state = "stop"
        self.reorient_flag = True

<<<<<<< HEAD
        # PID values
        self.ang_kp = 1
        self.ang_ki = 1
        self.ang_kd = 0  # change if doing PID, for now just PI

        self.lin_kp = 0
        self.lin_ki = 0
        self.lin_kd = 0

        self.errSum = 0
        self.last_error = 0
        self.motor_ang_ratio = 0  # the ratio between motor speeds

        # ROS stuff
        self.timer = self.create_timer(0.1, self.run_loop)

        self.pose_sub = self.create_subscription(
            Float32MultiArray, "/marker_pose", self.process_pose, 10
        )

    def angPID(self):
        """Run angular PID for the robot"""
        now = time.time()
        del_time = time.time() - self.last_iteration_time()

        # working variables
        error = self.current_target[0]
        self.errSum += error * del_time
        dErr = (error - self.last_error) / del_time

        # output
        self.motor_ang_ratio = (
            self.ang_kp * error + self.ang_ki * self.errSum + self.ang_kd + dErr
        )

        # remember
        self.last_error = error
        self.last_iteration_time = now

    def get_up_time(self):
        """get time since launch of program"""
        # is this even necessary? We want time between iterations for PID
        return time.time() - self.start_time()

    def run_loop(self):
        print(f"Running! State: {self.state}")
        match self.state:
            case "stop":
                self.drive_raw(0,0)
                return
            case "teleop":
                return
            case _:
                if self.new_target is not None:
                    y = self.new_target[0] 
                    if y < 0:
                        self.drive_raw(0,.4)
                    elif y > 0: 
                        self.drive_raw(.4,0)
                    else:
                        self.drive_raw(0,0)

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

    def heartbeat_callback(self, msg: Bool):
        self.latest_hb_time = self.get_clock().now().nanoseconds

    def process_twist(self, msg: Twist):
        # Change for actual units / something parameterizable
        print(f"I got a twist! {msg.angular.z} angular, {msg.linear.x} linear")
        ang = msg.angular.z
        lin = msg.linear.x
        if not (self.state == "stop" or self.state == "drive"):
            self.drive_raw(lin + ang, lin - ang)

    def process_teleop(self, msg: Int32):
        # Update state --- -1 is auto, teleop if equal to my id, stop if not
        val = msg.data
        self.teleop_state = val
        print(f"Received val: {val}")
        if val == self.id:
            self.state = "teleop"
        elif val == -1:
            self.state = "drive"
        else:
            self.state = "stop"

    def process_pose(self, msg):
        self.new_target = np.array(msg.data)
        self.last_detect_time = time.time()
        if self.current_target is None:
            return  # don't do calculations if None
        elif self.reorient_flag == False:
            pose_delta = np.linalg.norm(np.current_target - self.new_target)
            if pose_delta > self.pose_bound:
                self.reorient_flag = True


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(TestMotorSpin())
    rclpy.shutdown()
