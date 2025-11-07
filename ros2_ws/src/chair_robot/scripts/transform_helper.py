#!/usr/bin/env python3

import math
import os
import yaml
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from ament_index_python import get_package_share_directory

class StaticFrameBroadcaster(Node):
    """  Class to publish static frame transforms, loaded from file, when initialized """
    def __init__(self,path):
        super().__init__("static_frame_broadcast")
        self.broadcasters = []
        self.init_static_transforms(path)

    def init_static_transforms(self,config_path):
        # Load YAML options
        with open(config_path,'r') as f:
            config = yaml.safe_load(f)

        transforms = config["transforms"]
        print(transforms)
        tf_count = 0
        for parent in transforms:
            print(parent)
            for child in transforms[parent]:
                print(child)
                self.broadcasters.append(StaticTransformBroadcaster(self))
                self.make_transform(parent=parent,child=child,transform=transforms[parent][child],broadcast=self.broadcasters[tf_count])
                tf_count += 1

    def make_transform(self,parent,child,transform,broadcast):
        t = TransformStamped()

        quat = quaternion_from_euler(float(transform[3]),float(transform[4]),float(transform[5]))
        t.transform.rotation = quat
        t.transform.translation.x = float(transform[0])
        t.transform.translation.y = float(transform[1])
        t.transform.translation.z = float(transform[2])

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        broadcast.sendTransform(t)

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = Quaternion()
    q.x = cj*sc - sj*cs
    q.y = cj*ss + sj*cc
    q.z = cj*cs - sj*sc
    q.w = cj*cc + sj*ss

    return q

class FrameUpdater(Node):
    """ Class to publish frame transforms as a response to pose topic updates """
    def __init__(self,parent,child,topic):
        super().__init__('frame_updater')
        self.parent_frame = parent
        self.child_frame = child
        self.create_subscription(PoseStamped,topic,self.update_pose_callback,10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def update_pose_callback(self,msg):
        pt = msg.pose.position
        orientation = msg.pose.orientation

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = pt.x
        t.transform.translation.y = pt.y
        t.transform.translation.z = pt.z
        t.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(t)

if __name__ == "__main__":
    rclpy.init()
    share_dir = get_package_share_directory("chair_robot")
    config_path = os.path.join(share_dir,"config","marker_ids.yaml")
    #print(f"share dir: {share_dir}")
    #print(f"config path: {config_path}")
    rclpy.spin(StaticFrameBroadcaster(config_path))
    rclpy.shutdown()