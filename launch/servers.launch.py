import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

def generate_launch_description():
    package_name = 'rb1_shelf_tools'

    #~~~~~~~~~~~~~~~~~~Declare path~~~~~~~~~~~~~~~
    config_file = os.path.join(get_package_share_directory(package_name),'config','config.yaml')

    return LaunchDescription([

        #~~~~~~~~~~~~~~~~~~Server find shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package= 'rb1_shelf_tools',
            executable='find_shelf_service_server_node',
            name='find_shelf_service_server',
            output='screen'
            ),

        #~~~~~~~~~~~~~~~~~~Server approach shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='rb1_shelf_tools',
            executable='approach_shelf_service_server_node',
            name='approach_shelf_service_server',
            output='screen',
            parameters=[config_file]
        )

        ]) 