from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    video_device_str = LaunchConfiguration("device").perform(context)
    video_device = int(video_device_str)

    nodes_to_launch = [
        Node(
            package='chair_robot',
            executable='robot_state.py',
            name='robot0',
            namespace='robot0',
            parameters=[
                {"device":video_device},
                {"is_trashcan":True},
                {"follow_id":-1},
                {"id":0}
            ]
        ),
        Node(
            package='chair_robot',
            executable='transform_helper.py',
            name='static_broadcast',
        )
    ]
    
    return nodes_to_launch

def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        "device",
        default_value='4',
        description="ID of video device to use"
    )
    return LaunchDescription([video_device_arg, OpaqueFunction(function=launch_setup)])