import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():

    package_name = 'diff_bot'

    sim_mode = LaunchConfiguration('sim_mode')

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'config.rviz'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=LaunchConfigurationEquals('sim_mode', 'rviz'),
    )

    node_foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=LaunchConfigurationEquals('sim_mode', 'foxglove'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='rviz',
            description='Simulation mode: rviz or foxglove'
        ),
        node_rviz,
        node_foxglove,
    ])

