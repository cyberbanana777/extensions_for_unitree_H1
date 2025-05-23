from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="buttons_server",
            executable="server_stand_up_without_hands"
        ),
        Node(
            package="buttons_server",
            executable="hear_server"
        ),
        Node(
            package="button_analyzer",
            executable="button_analyzer_without_hands"
        ),
        Node(
            package="low_level_control",
            executable="low_level_control_without_hands"
        )


    ])
