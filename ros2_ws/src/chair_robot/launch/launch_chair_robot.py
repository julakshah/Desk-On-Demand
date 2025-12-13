from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux',
            exectuable='twist_mux',
            name='twist_mux',
            parameters=['config/twist_mux.yaml']
        )
        Node(
            package='chair_robot',
            exectuable='transform_helper',
            name='transform_helper'
            )
        Node(
            package='chair_robot',
            exectuable='robot_state',
            name='state_machine'
            parameters=[{'is_trashcan': False}]
            )
        ]) 
