import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    description = []

    verbosity_level = DeclareLaunchArgument(name='log_level', default_value='info')

    DeadzoneActionServer = Node(
        package= 'sycabot_identification',
        executable= 'DeadzoneActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml',
                    {"id": LaunchConfiguration('id')},],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    IdentificationActionServer = Node(
        package= 'sycabot_identification',
        executable= 'IdentificationActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml',
                    {"id": LaunchConfiguration('id')},],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    MPCActionServer = Node(
        package= 'sycabot_control',
        executable= 'MPCActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml',
                    {"id": LaunchConfiguration('id')},],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    PPCtrllerActionServer = Node(
        package= 'sycabot_control',
        executable= 'PPCtrllerActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml',
                    {"id": LaunchConfiguration('id')},],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    beaconClient = Node(
        package= 'sycabot_base',
        executable= 'advertise_id',
        output = 'screen',
        emulate_tty=True,
       parameters=['config/params_identification.yaml',
                    {"id": LaunchConfiguration('id')},],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    description.append(verbosity_level)
    description.append(IdentificationActionServer)
    description.append(DeadzoneActionServer)
    description.append(MPCActionServer)
    description.append(PPCtrllerActionServer)
    description.append(beaconClient)
    return LaunchDescription(description)