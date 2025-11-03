# ROS2 nodes
import rclpy
from std_msgs.msg import String
from rclpy.node import Node

# Pi gpio control
from gpiozero import Motor
from time import sleep

class TestMotorSpin(Node):
    def __init__(self):
        self.motor = Motor(forward=4, backward=14)
        self.create_timer(0.1, self.run_loop)


        self.new_target = None
        self.current_target = None
        self.pose_sub = self.create_subscription(Float32MultiArray, "marker_pose", self.process_pose, 10)
        self.pose_bound = 2
        self.reorient_flag = True


    def run_loop(self):
        motor.forward()
        sleep(5)
        motor.backward()
        sleep(5)

    def process_pose(self, msg):
        self.new_target = np.array(msg.data)
        if self.current_target is None:
            return #don't do calculations if None
        elif self.reorient_flag == False:
            pose_delta = np.linalg.norm(np.current_target - self.new_target)
            if pose_delta > self.pose_bound:
                self.reorient_flag = True

        

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(PubTest())
    rclpy.shutdown()
