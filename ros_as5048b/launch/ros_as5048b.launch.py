from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    i2c_device_arg = DeclareLaunchArgument('i2c_device', default_value="1")
    i2c_address_arg = DeclareLaunchArgument('i2c_address', default_value="0x42")

    encoder_topic_arg = DeclareLaunchArgument('encoder_topic', default_value="/encoder/at_0x42/deg")
    turn_count_file_arg = DeclareLaunchArgument('turn_count_file', default_value='/tmp/turn_count.yaml')

    read_encoder_node = Node(
            package='ros_as5048b',
            executable='read_encoder_node',
            name='read_encoder_node',
            parameters=[
                {'i2c_device': LaunchConfiguration('i2c_device')},
                {'i2c_address': LaunchConfiguration('i2c_address')},
            ]    
         )

    turn_counter_node = Node(
            package='ros_as5048b',
            executable='turn_counter_node',
            name='turn_counter_node',
            parameters=[
                {'file_path': LaunchConfiguration('turn_count_file')},
                {'encoder_topic': LaunchConfiguration('encoder_topic')},
            ]    
         )

    nodes = [
        read_encoder_node,
        turn_counter_node
    ]

    return LaunchDescription([
        i2c_device_arg,
        i2c_address_arg,
        encoder_topic_arg,
        turn_count_file_arg,
    ] + nodes)



