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

    DeadzoneActionServer = Node(
        package= 'sycabot_identification',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'DeadzoneActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    IdentificationActionServer = Node(
        package= 'sycabot_identification',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'IdentificationActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    MPCActionServer = Node(
        package= 'sycabot_control',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'MPCActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    PPCtrllerActionServer = Node(
        package= 'sycabot_control',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'PPCtrllerActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    motors = Node(
        package= 'sycabot_base',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'motors',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    beaconClient = Node(
        package= 'sycabot_base',
        executable= 'advertise_id',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    description.append(verbosity_level)
    description.append(IdentificationActionServer)
    description.append(DeadzoneActionServer)
    description.append(MPCActionServer)
    description.append(PPCtrllerActionServer)
    description.append(motors)
    description.append(beaconClient)
    return LaunchDescription(description)