from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

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

    return LaunchDescription([
        serial_bridge,
        upper_filter,
        forearm_filter
    ])