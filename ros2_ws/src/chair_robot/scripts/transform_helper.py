#!/usr/bin/env python3

import math
import os
import yaml
import numpy as np
from geometry_msgs.msg import TransformStamped, Transform, PoseStamped, Quaternion
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformListener, Buffer
from tf_transformations import inverse_matrix, quaternion_matrix, quaternion_from_matrix
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

        # For now, we have no global world --> target can be aligned with it always
        # would have to change if we had some global world
        tf_target = TransformStamped()
        tf_target.child_frame_id = "robot0"
        tf_target.header.frame_id = "world"
        tf_target.header.stamp = self.get_clock().now().to_msg()
        target_broadcast = StaticTransformBroadcaster(self)
        target_broadcast.sendTransform(tf_target)
        print(f"Broadcast target transform")


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
    def __init__(self,node: Node,follow_target,parent,child,id):
        self.node = node
        self.parent_frame = parent # frame we want to update
        self.child_frame = child # world
        self.id = id
        self.ids = []

        share_dir = get_package_share_directory("chair_robot")
        config_path = os.path.join(share_dir,"config","marker_ids.yaml")
        with open(config_path, 'r')as f:
            config = yaml.safe_load(f)
        print(f"Config: {config}")

        self.known_ids = config["ids"] # map tag ID to robot ID
        self.robot_names = config["names"] # map robot ID to robot name
        self.name_to_id = {v:k for k,v in self.robot_names.items()}
        self.follow_target = follow_target # robot ID

        # Topic which holds all pose updates
        self.transform_sub = self.node.create_subscription(TransformStamped,"/pose_updates",self.update_pose_callback,10)

        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self.node)

        self.transforms = {}
        self.distances = {}

        #self.tf_pub = self.node.create_publisher(TransformStamped,"/pose_updates",10)

        # Timer to read encoder data
        self.encoder_timer = self.node.create_timer(0.01,self.read_encoder)
        #self.encoder_pin_l = PWMLED(0)
        #self.encoder_pin_r = PWMLED(0)
    
    def read_transforms(self):
        for id in self.known_ids.values():
            if id != self.id:
                print(f"I'm trying {id} and {self.id} (going to lookup {self.parent_frame} and {self.robot_names[id]})")
                try:
                    t = self.tf_buffer.lookup_transform(
                        self.parent_frame,
                        self.robot_names[id],
                        rclpy.time.Time(),
                    )
                except:
                    print(f"Unable to get transform between {self.robot_names[id]} to my frame {self.parent_frame}")
                    continue
                print(f"Got transform from robot {self.robot_names[id]} to my own frame, {self.parent_frame}")
                self.transforms[id] = t
                translation = t.transform._translation
                self.distances[id] = np.sqrt(translation.x**2 + translation.z**2)
                #print(f"Got distance from {self.parent_frame} to {self.robot_names[id]} as {self.distances[id]}")


    def read_encoder(self):
        data_l = 0

    def update_pose_callback(self,msg: TransformStamped):
        frame_p = msg.header.frame_id 
        frame_c = msg.child_frame_id

        #print(f"self name to id: {self.name_to_id}")
        p_id = self.name_to_id[frame_p]  #1 
        c_id = self.name_to_id[frame_c]  #2

        print(f"My frame parent is {self.parent_frame}, looking at {frame_p} and {frame_c}")
        
        if frame_p == self.parent_frame:
            # if we're higher id than the other, we do the update
            if p_id > c_id:
                print(f"Updating {frame_p} because I'm {self.parent_frame} and {p_id} > {c_id}")
                # We pass the frame we're not updating
                self.publish_new_transform(frame_c,msg.transform,False)

        elif frame_c == self.parent_frame:
            if c_id > p_id:
                print(f"Updating {frame_c} because I'm {self.parent_frame} and {c_id} > {p_id}")
                self.publish_new_transform(frame_p,msg.transform,True)
                return
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

    def publish_new_transform(self,to_frame,pose: Transform,inverse):
        # The target is always the world frame (world has no meaning to us)
        try:
            t0 = self.tf_buffer.lookup_transform(
                "world", # world to to_frame
                to_frame,
                rclpy.time.Time())
        except:
            print(f"Could not get transform to update (tried {to_frame} and world)")
            t0 = TransformStamped()
            t0.header.frame_id = "world"
            t0.child_frame_id = to_frame
            t0.transform.translation.x = 0.0
            t0.transform.translation.y = 0.0
            t0.transform.translation.z - 0.0

        #to_frame->parent_frame = to_frame->world * world->parent_frame
        #world->parent_frame = world->to_frame * to_frame->parent_frame

        t = TransformStamped()
        t.header.frame_id = self.child_frame
        t.child_frame_id = self.parent_frame
        t.transform.translation.x = pose.translation.x
        t.transform.translation.y = pose.translation.y
        t.transform.translation.z = pose.translation.z
        t.transform.rotation = pose.rotation
        print(f"t.transform.rotation: {t.transform.rotation}")

        if inverse:
            t.transform = mat_to_transform(inverse_matrix(tf_to_mat(t))).transform

        t_world_to_parent = mat_to_transform(tf_to_mat(t0) @ tf_to_mat(t)).transform
        ts_world_to_parent = TransformStamped()
        ts_world_to_parent.header.stamp = self.node.get_clock().now().to_msg()
        ts_world_to_parent.header.frame_id = "world"
        ts_world_to_parent.child_frame_id = self.parent_frame
        ts_world_to_parent.transform = t_world_to_parent

        print(f"Publishing transform: {ts_world_to_parent}\n\nIts components were: {t0}, {t}.\n Used world to {to_frame} for t0")

        # Publish transform
        self.tf_broadcaster.sendTransform(ts_world_to_parent)
        
def tf_to_mat(T: TransformStamped):
    t = T.transform.translation
    q = T.transform.rotation
    M = quaternion_matrix([q.x, q.y, q.z, q.w])
    M[0, 3] = t.x
    M[1, 3] = t.y
    M[2, 3] = t.z
    return M


def mat_to_transform(M: np.ndarray):
    out = TransformStamped()
    out.transform.translation.x = float(M[0, 3])
    out.transform.translation.y = float(M[1, 3])
    out.transform.translation.z = float(M[2, 3])

    qx, qy, qz, qw = quaternion_from_matrix(M)
    out.transform.rotation.x = float(qx)
    out.transform.rotation.y = float(qy)
    out.transform.rotation.z = float(qz)
    out.transform.rotation.w = float(qw)
    return out


if __name__ == "__main__":
    rclpy.init()
    share_dir = get_package_share_directory("chair_robot")
    config_path = os.path.join(share_dir,"config","marker_ids.yaml")
    #print(f"share dir: {share_dir}")
    #print(f"config path: {config_path}")
    rclpy.spin(StaticFrameBroadcaster(config_path))
    rclpy.shutdown()