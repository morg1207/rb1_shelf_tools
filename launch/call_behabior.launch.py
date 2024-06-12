import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable

def generate_launch_description():
    package_name = 'rb1_shelf_tools'

    #~~~~~~~~~~~~~~~~~~Declare path~~~~~~~~~~~~~~~
    config_behavior = os.path.join(get_package_share_directory(package_name),'config','config_behavior.yaml')

    return LaunchDescription([

        #~~~~~~~~~~~~~~~~~~Server find shelf~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package= 'rb1_shelf_tools',
            executable='bt_ros2_async',
            name='bt_ros2_async',
            output='screen',
            parameters=[config_behavior]
            ),


        ]) 