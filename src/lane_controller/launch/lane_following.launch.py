from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    lane_perception = Node(
        package="lane_perception",
        executable="lane_detector",
        name="lane_detector",
        output="screen"
    )

    lane_controller = Node(
        package="lane_controller",
        executable="lane_controller",
        name="lane_controller",
        output="screen"
    )

    return LaunchDescription([
        lane_perception,
        lane_controller
    ])