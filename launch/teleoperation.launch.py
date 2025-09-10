import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'diff_bot'

    sim_mode = LaunchConfiguration('sim_mode')
    real_time = LaunchConfiguration('real_time')

    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/include' ,'sim.launch.py')]), 
            launch_arguments={'sim_mode': sim_mode}.items()
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/include' ,'vel_controllers.launch.py')]), 
            launch_arguments={'cmd_mode': 'speed', 'real_time': real_time}.items() # set cmd_mode to speed for velocity control
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='foxglove',
            description='Simulation mode: rviz or foxglove'),

        DeclareLaunchArgument(
            'real_time',
            default_value='false',
            description='Enable real-time control'),
            
        spawn_controllers,
        sim_node,
    ])