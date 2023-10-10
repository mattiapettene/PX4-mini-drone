from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gps_py",
            executable="gps_velocity_py",
            name="gps_velocity_py",
            output="screen",
            emulate_tty=True,
        )
    ])