import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('project_launch'))
    map_file = os.path.join(config_dir,'my_map4.yaml')
    param_file = os.path.join(config_dir,'nav2_config.yaml')


    return LaunchDescription([


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': param_file}.items(),

        ),


   
    ])
