import math
import yaml
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class StaticFrameBroadcaster(Node):
    """  Class to publish static frame transforms, loaded from file, when initialized """
    def __init__(self,path):
        super().__init__("static_frame_broadcast")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.init_static_transforms(path)

    def init_static_transforms(self,config_path):
        # Load YAML options
        with open(config_path,'r') as f:
            config = yaml.safe_load(f)

        transforms = config.transforms
        for parent in transforms:
            for child in parent:
                self.make_transform(parent=parent,child=child,transform=transforms[parent][child])

    def make_transform(self,parent,child,transform):
        t = TransformStamped()

        quat = quaternion_from_euler(float(transform[3]),float(transform[4]),float(transform[5]))
        t.transform.rotation = quat
        t.transform.translation.x = float(transform[0])
        t.transform.translation.y = float(transform[1])
        t.transform.translation.z = float(transform[2])

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        self.broadcaster.sendTransform(t)

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

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

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