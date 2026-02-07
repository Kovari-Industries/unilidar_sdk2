import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # CropBox parameters (defaults from nav2_params robot footprint)
    crop_min_x_arg = DeclareLaunchArgument('crop_min_x', default_value='-0.70')
    crop_max_x_arg = DeclareLaunchArgument('crop_max_x', default_value='0.09')
    crop_min_y_arg = DeclareLaunchArgument('crop_min_y', default_value='-0.35')
    crop_max_y_arg = DeclareLaunchArgument('crop_max_y', default_value='0.34')
    crop_min_z_arg = DeclareLaunchArgument('crop_min_z', default_value='-0.50')
    crop_max_z_arg = DeclareLaunchArgument('crop_max_z', default_value='0.50')

    # Lidar node
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 2},
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
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
        ],
    )

    # CropBox filter to remove robot body points
    cropbox_filter = Node(
        package='unitree_lidar_ros2',
        executable='pointcloud_cropbox_filter',
        name='pointcloud_cropbox_filter',
        output='screen',
        parameters=[
            {'input_topic': 'unilidar/cloud'},
            {'output_topic': 'unilidar/cloud_filtered'},
            {'crop_min_x': LaunchConfiguration('crop_min_x')},
            {'crop_max_x': LaunchConfiguration('crop_max_x')},
            {'crop_min_y': LaunchConfiguration('crop_min_y')},
            {'crop_max_y': LaunchConfiguration('crop_max_y')},
            {'crop_min_z': LaunchConfiguration('crop_min_z')},
            {'crop_max_z': LaunchConfiguration('crop_max_z')},
            {'negative': True},
            {'target_frame': 'lidar_center'},
        ],
    )

    # RViz
    package_path = subprocess.check_output(
        ['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']
    ).decode('utf-8').rstrip()
    rviz_config_file = os.path.join(
        package_path, 'share', 'unitree_lidar_ros2', 'view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='log',
    )

    return LaunchDescription([
        crop_min_x_arg,
        crop_max_x_arg,
        crop_min_y_arg,
        crop_max_y_arg,
        crop_min_z_arg,
        crop_max_z_arg,
        lidar_node,
        cropbox_filter,
        rviz_node,
    ])
