import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    description = []
    id = DeclareLaunchArgument('id', default_value=TextSubstitution(text="1"))
    description.append(id)

    description.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sycabot_launch'),
                'robot.launch.py')),
        launch_arguments = {"id":  LaunchConfiguration('id')}.items(),
    ))
    
    return LaunchDescription(description)