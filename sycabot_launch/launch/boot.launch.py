from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


SYCABOT_ID = 1

def generate_launch_description():

    description = []

    DeadzoneActionServer = Node(
        package= 'sycabot_identification',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'DeadzoneActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml']
    )

    IdentificationActionServer = Node(
        package= 'sycabot_identification',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'IdentificationActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml']
    )

    MPCActionServer = Node(
        package= 'sycabot_control',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'MPCActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
    )

    PPCtrllerActionServer = Node(
        package= 'sycabot_control',
        namespace= f'SycaBot_W{SYCABOT_ID}',
        executable= 'PPCtrllerActionServer',
        output = 'screen',
        emulate_tty=True,
        parameters=['config/params_identification.yaml'],
    )
    motors = Node(
        package= 'sycabot_base',
        namespace= 'SycaBot_W' + str(SYCABOT_ID),
        executable= 'motors',
        output = 'screen',
        emulate_tty=True,
    )
    description.append(IdentificationActionServer)
    description.append(DeadzoneActionServer)
    description.append(MPCActionServer)
    description.append(PPCtrllerActionServer)
    description.append(motors)
    return LaunchDescription(description)