#!/usr/bin/env python3

import math
import os
import yaml
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener, Buffer
from tf_transformations import inverse_matrix
from gpiozero import PWMLED, LED

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

class FrameUpdater:
    """ Class to publish frame transforms as a response to pose topic updates """
    def __init__(self,node: Node,parent,child,id):
        self.node = node
        self.parent_frame = parent # frame we want to update
        self.child_frame = child # world
        self.id = id

        # Topic which holds all pose updates
        self.transform_sub = self.node.create_subscription(TransformStamped,"/pose_updates",self.update_pose_callback,10)

        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        #self.tf_pub = self.node.create_publisher(TransformStamped,"/pose_updates",10)

        # Timer to read encoder data
        self.encoder_timer = self.node.create_timer(0.01,)
        self.encoder_pin_l = PWMLED(0)
        self.encoder_pin_r = PWMLED(0)

    def read_encoder(self):
        data_l = 0

    def update_pose_callback(self,msg: TransformStamped):
        frame_p = msg.header.frame_id 
        frame_c = msg.child_frame_id

        if frame_p == self.parent_frame:
            # if we're greater id than the other, we do the update
            if frame_p > frame_c:
                self.publish_new_transform(frame_c,msg.pose,False)

        elif frame_c == self.parent_frame:
            if frame_c > frame_p:
                self.publish_new_transform(frame_p,msg.pose,True)

                pt = msg.transform.translation
                orientation = msg.transform.rotation

                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = self.parent_frame
                t.child_frame_id = self.child_frame
                t.transform.translation = pt
                t.transform.rotation = orientation

                self.node.get_logger().info(f"Update frame: {self}, translation: {t.transform.translation}, rot: {t.transform.rotation}")
                self.tf_broadcaster.sendTransform(t)

    def publish_new_transform(self,to_frame,pose,inverse):
        t = self.tf_buffer.lookup_transform(
            "world",
            to_frame,
            rclpy.time.Time())
        # parent_frame->to_frame = parent_frame->world * world->to_frame
        # parent_frame->world = parent_frame->to_frame * to_frame->world
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.positiom.y
        t.transform.translation.z = pose.positiom.z
        t.transform.rotation = pose.orientation

        if inverse:
            t.transform = inverse_matrix(t.transform)

        # Publish transform
        self.tf_broadcaster.sendTransform(t)



if __name__ == "__main__":
    rclpy.init()
    share_dir = get_package_share_directory("chair_robot")
    config_path = os.path.join(share_dir,"config","marker_ids.yaml")
    #print(f"share dir: {share_dir}")
    #print(f"config path: {config_path}")
    rclpy.spin(StaticFrameBroadcaster(config_path))
    rclpy.shutdown()