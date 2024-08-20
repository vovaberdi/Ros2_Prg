from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topics_quiz_pkg',
            executable='topics_quiz',
            output='screen'),
    ])
