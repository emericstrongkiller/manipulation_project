import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_config_dir = "/home/user/ros2_ws/src/manipulation_project/object_detection/config/rviz_perception_config.rviz"

    return LaunchDescription([
        Node(
            package='object_detection',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
            ),
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen',
            parameters=[{'use_sim_time': True}]),
        Node(
            package='moveit2_scripts',
            executable='pick_and_place_perception',
            name='pick_and_place_perception',
            output='screen',
            parameters=[{'use_sim_time': True}])
        ])