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
        ),
        Node(
            package="qualisys_ros",
            executable="qualisys_node",
            name="qualisys_node",
            output="screen",
            emulate_tty=True,
            arguments=[
                "--server", "192.168.1.216",
                "--rate", "100",
            ],
        )
    ])