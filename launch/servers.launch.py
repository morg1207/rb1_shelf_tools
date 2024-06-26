import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'rb1_shelf_tools'

    #~~~~~~~~~~~~~~~~~~Declare path~~~~~~~~~~~~~~~
    config_find_file = os.path.join(get_package_share_directory(package_name),'config','config_find.yaml')
    config_approach_file = os.path.join(get_package_share_directory(package_name),'config','config_approach.yaml')
    config_init_localization_file = os.path.join(get_package_share_directory(package_name),'config','config_init_localization.yaml')
    config_find_nav_poses_file = os.path.join(get_package_share_directory(package_name),'config','config_find_nav_poses.yaml')

    return LaunchDescription([

        #~~~~~~~~~~~~~~~~~~Server find shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package= 'rb1_shelf_tools',
            executable='find_shelf_service_server_node',
            name='find_shelf_service_server',
            output='screen',
            parameters=[config_find_file],
            #arguments=['--ros-args', '--log-level', 'find_shelf_service_server:=DEBUG'],
            ),

        #~~~~~~~~~~~~~~~~~~Server approach shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='rb1_shelf_tools',
            executable='approach_shelf_service_server_node',
            name='approach_shelf_service_server',
            output='screen',
            #arguments=['--ros-args', '--log-level', 'approach_shelf_service_server:=DEBUG'],
            parameters=[config_approach_file]
        ),
        #~~~~~~~~~~~~~~~~~~Server localization~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='rb1_shelf_tools',
            executable='init_localization_service_server_node',
            name='init_localization_service_server',
            output='screen',
            parameters=[config_init_localization_file],
            #arguments=['--ros-args', '--log-level', 'init_localization_service_server:=DEBUG'],
        ),
        #~~~~~~~~~~~~~~~~~~filter laser~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='filter_map_laser',
            executable='filter_lidar_to_map',
            name='filter_lidar_to_map',
            output='screen',
        ),
        #~~~~~~~~~~~~~~~~~~Server deep found shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='filter_map_laser',
            executable='detect_legs_in_map_server',
            name='detect_legs_in_map_server',
            output='screen',
        ),
        #~~~~~~~~~~~~~~~~~~Server nav poses~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='rb1_shelf_tools',
            executable='find_nav_points_service_server_node',
            name='find_nav_points_service_server',
            parameters=[config_find_nav_poses_file],
            output='screen',
            arguments=['--ros-args', '--log-level', 'find_nav_points_service_server:=DEBUG'],
        ),
        #~~~~~~~~~~~~~~~~~~Laser filters~~~~~~~~~~~~~~~~~~~~~~~~~~
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("laser_filters"), '/examples', '/shadow_filter_example.launch.py'])
            )
        ]) 