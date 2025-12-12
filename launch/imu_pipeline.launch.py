from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package info for usb cam and ros2_aruco
    usb_cam_pkg = get_package_share_directory('usb_cam')
    aruco_pkg = get_package_share_directory('ros2_aruco')

    # Camera for hand and aruco detection
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(usb_cam_pkg, 'launch', 'camera.launch.py'))
    )

    # Publishes aruco poses
    # aruco_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(aruco_pkg, 'launch', 'aruco_recognition.launch.py'))
    # )

    # Determines transform between 2 aruco tags
    # rrs_transform_node = Node(
    #     package='rrs_transform',
    #     executable='rrs_transform',
    #     name='rrs_transform_node',
    #     parameters=[{
    #         'body_base_link_frame': 'ar_marker_1',
    #         'body_tool_frame': 'ar_marker_10',
    #     }]
    # )

    # Detects when hand is open or closed
    clasp_detection_node = Node(
        package='rrs_transform',
        executable='clasp_detection',
        name='clasp_detection_node'
    )

    # Serial IMU bridge node (publishes raw IMU topics)
    serial_bridge = Node(
        package='imu_pipeline',
        executable='serial_imu_bridge',
        name='serial_imu_bridge',
        output='screen',
    )

    # Upper arm Madgwick filter (publishes upper arm orientation quaternions)
    upper_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='upper_filter',
        output='screen',
        remappings=[
            ('imu/data_raw', '/upper_imu/data_raw'),
            ('imu/data', '/upper_imu/data'),
        ],
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu'
        }]
    )

    # Forearm Madgwick filter (publishes forearm orientation quaternions)
    forearm_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='forearm_filter',
        output='screen',
        remappings=[
            ('imu/data_raw', '/forearm_imu/data_raw'),
            ('imu/data', '/forearm_imu/data'),
        ],
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu'
        }]
    )

    # Punching node (publishes joint angles to UR7e using quaternions)
    punching_node = Node(
        package='imu_pipeline',          
        executable='punch',
        name='punch',
        output='screen',
    )

    return LaunchDescription([
        camera_launch,
        # aruco_launch,
        # rrs_transform_node,
        clasp_detection_node,
        serial_bridge,
        upper_filter,
        forearm_filter,
        punching_node
    ])