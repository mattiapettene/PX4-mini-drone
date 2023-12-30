from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bridge_uwb_px4",
            executable="uwb_bridge",
            name="uwb_bridge",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="bridge_uwb_px4",
            executable="mb1202_bridge",
            name="mb1202_bridge",
            output="screen",
            emulate_tty=True,
        )
    ])