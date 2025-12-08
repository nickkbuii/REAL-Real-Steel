from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    upper_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="upper_imu_filter",
        parameters=["config/upper_filter.yaml"],
        remappings=[
            ("imu/data_raw", "upper_imu/data_raw"),
            ("imu/data",     "upper_imu/data"),
        ]
    )

    forearm_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="forearm_imu_filter",
        parameters=["config/forearm_filter.yaml"],
        remappings=[
            ("imu/data_raw", "forearm_imu/data_raw"),
            ("imu/data",     "forearm_imu/data"),
        ]
    )

    return LaunchDescription([upper_filter, forearm_filter])