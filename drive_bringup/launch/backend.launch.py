from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    hardware_pkg_share = get_package_share_directory("phidgets_hardware")
    description_pkg_share = get_package_share_directory("drive_description")
    xacro_path = os.path.join(description_pkg_share, "description", "phidgets_giskard.urdf.xacro")
    controllers_path = os.path.join(hardware_pkg_share, "config", "ros2_control_controllers.yaml")

    robot_description = xacro.process_file(xacro_path).toxml() 
  
    hardware_backend = Node(
        package="rover_dashboard",
        executable="hardware_backend",
        name="rover_dashboard",
        output="screen"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_path,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )


    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--activate"],
        output="screen",
    )


    motor_position_node = Node(
        package="drive_control",
        executable="motor_position_publisher",
        output="screen",
    )

    jetson_relay = Node(
        package="drive_control",
        executable="jetson_relay",
        output="screen",
    )

    return LaunchDescription([
        hardware_backend, 
        ros2_control_node, 

        robot_state_publisher, 
        spawn_jsb,
        motor_position_node, 
        jetson_relay
    ])