from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os

def generate_launch_description():

    camera_publisher_node = Node(
        package='camera_publishers',
        namespace='camera_publishers',
        executable='camera_publishers'
        
    )

    object_avoidance_node = Node(
       package='object_avoidance',
       namespace='object_avoidance',
       executable='object_avoidance_node',
       remappings=[
                ('/object_detection/cmd_vel', '/twist_mux/cmd_vel'),
            ],
    )

    # object_derek_node = Node(
    #    package='object_derek',
    #    namespace='object_derek',
    #    executable='object_derek_node',
    #    # remappings=[
    #    #          ('/object_detection/cmd_vel', '/twist_mux/cmd_vel'),
    #     #     ],
    # )

     #teleop_twist_keyboard_node = Node(
    #    package='teleop_twist_keyboard',
    #    namespace='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard'
    # 	 )

    web_video_server_node = Node(
        package='web_video_server',
        namespace='web_video_server',
        executable='web_video_server'
        
    )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('realsense2_camera'),
        #             'launch/rs_launch.py'
        #         ])
        #     ])              
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('twist_mux'),
        #             'launch/twist_mux_launch.py'
        #         ])
        #     ])
        # ),
         camera_publisher_node,
         object_avoidance_node,
         web_video_server_node
        #  object_derek_node
        ])