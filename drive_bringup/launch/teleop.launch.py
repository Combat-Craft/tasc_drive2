from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen"
    )

    teleop = Node(
        package="phidgets_hardware",
        executable="ps4_teleop",
        name="ps4_teleop",
        output="screen"
    )

    dashboard = Node(
        package="rover_dashboard",
        executable="dashboard",
        name="rover_dashboard",
        output="screen"
    )

    return LaunchDescription([
        joy,
        teleop,
        dashboard
    ])