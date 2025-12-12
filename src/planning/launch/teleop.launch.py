from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to included launch files
    usb_cam_pkg = get_package_share_directory('usb_cam')
    aruco_pkg = get_package_share_directory('ros2_aruco')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(usb_cam_pkg, 'launch', 'camera.launch.py'))
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aruco_pkg, 'launch', 'aruco_recognition.launch.py'))
    )

    # DeclareLaunchArgument(
    #     'body_base_link_frame',
    #     default_value='ar_marker_1',
    # ),
    # DeclareLaunchArgument(
    #     'body_tool_frame',
    #     default_value='ar_marker_10',
    # ),

    # rrs_transform_node = Node(
    #     package='rrs_transform',
    #     executable='rrs_transform',
    #     name='rrs_transform_node',
    #     parameters=[
    #         {
    #             'body_base_link_frame': LaunchConfiguration('body_base_link_frame'), 
    #             'body_tool_frame': LaunchConfiguration('body_tool_frame'),
    #         }
    #     ]
    # )

    clasp_detection_node = Node(
        package='rrs_transform',
        executable='clasp_detection',
        name='clasp_detection_node'
    )

    ik_node = Node(
        package='planning',
        executable='ik',
        name='ik_node',
    )

    ur_type = LaunchConfiguration("ur_type", default="ur7e")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    moveit_launch_file = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py"
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz
        }.items(),
    )

    return LaunchDescription([
        camera_launch,
        aruco_launch,
        # rrs_transform_node,
        clasp_detection_node,
        moveit_launch,
    ])
