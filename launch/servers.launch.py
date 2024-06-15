import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

def generate_launch_description():
    package_name = 'rb1_shelf_tools'

    #~~~~~~~~~~~~~~~~~~Declare path~~~~~~~~~~~~~~~
    config_find_file = os.path.join(get_package_share_directory(package_name),'config','config_find.yaml')
    config_approach_file = os.path.join(get_package_share_directory(package_name),'config','config_approach.yaml')
    config_init_localization_file = os.path.join(get_package_share_directory(package_name),'config','config_init_localization.yaml')


    return LaunchDescription([

        #~~~~~~~~~~~~~~~~~~Server find shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package= 'rb1_shelf_tools',
            executable='find_shelf_service_server_node',
            name='find_shelf_service_server',
            output='screen',
            parameters=[config_find_file]
            ),

        #~~~~~~~~~~~~~~~~~~Server approach shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='rb1_shelf_tools',
            executable='approach_shelf_service_server_node',
            name='approach_shelf_service_server',
            output='screen',
            parameters=[config_approach_file]
        ),
        #~~~~~~~~~~~~~~~~~~Server approach shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='rb1_shelf_tools',
            executable='init_localization_service_server_node',
            name='init_localization_service_server',
            output='screen',
            parameters=[config_init_localization_file],
            arguments=['--ros-args', '--log-level', 'init_localization_service_server:=DEBUG'],
        )
        ]) 