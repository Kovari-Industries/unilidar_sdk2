import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Base frame of the robot (parent frame for lidar transform)'
    )
    
    # Transform arguments - Measured lidar position relative to base_link
    # x: 12cm forward, y: -10cm (negative y), z: 17cm up
    x_arg = DeclareLaunchArgument('x', default_value='0.12', description='X offset in meters (forward)')
    y_arg = DeclareLaunchArgument('y', default_value='-0.10', description='Y offset in meters (negative y direction)')
    z_arg = DeclareLaunchArgument('z', default_value='0.17', description='Z offset in meters (up)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw rotation in radians')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch rotation in radians')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0', description='Roll rotation in radians')
    
    # --- Unitree Lidar Driver ---
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'initialize_type': 2},
            {'work_mode': 0},
            {'use_system_timestamp': True},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 18},

            {'serial_port': '/dev/ttyACM0'},
            {'baudrate': 4000000},

            {'lidar_port': 6101},
            {'lidar_ip': '192.168.1.62'},
            {'local_port': 6201},
            {'local_ip': '192.168.1.2'},
            
            {'cloud_frame': "unilidar_lidar"},
            {'cloud_topic': "unilidar/cloud"},
            {'imu_frame': "unilidar_imu"},
            {'imu_topic': "unilidar/imu"},
        ]
    )

    # --- Static Transform Publisher ---
    # Connects base_link (or your robot's base frame) to the lidar frame
    # Using new-style arguments with flags (ROS 2 format)
    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=[
            '--x', LaunchConfiguration('x'),
            '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'),
            '--yaw', LaunchConfiguration('yaw'),
            '--pitch', LaunchConfiguration('pitch'),
            '--roll', LaunchConfiguration('roll'),
            '--frame-id', LaunchConfiguration('base_frame'),  # parent frame
            '--child-frame-id', 'unilidar_lidar'              # child frame (lidar)
        ]
    )

    return LaunchDescription([
        base_frame_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        pitch_arg,
        roll_arg,
        lidar_node,
        lidar_tf_publisher,
    ])






