import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


SYCABOT_ID = 1

def generate_launch_description():

    description = []

    verbosity_level = DeclareLaunchArgument(name='log_level', default_value='info')

    motors = Node(
        package= 'sycabot_base',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'motors',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    description.append(verbosity_level)
    description.append(motors)
    return LaunchDescription(description)