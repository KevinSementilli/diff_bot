import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    package_name = 'diff_bot'

    cmd_mode = LaunchConfiguration('cmd_mode')
    real_time = LaunchConfiguration('real_time')

    # launch robot_state_publisher 
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch/include' ,'rsp.launch.py')]), 
            launch_arguments={'cmd_mode': cmd_mode, 'real_time': real_time}.items()
    )

    controller_config = os.path.join(
        get_package_share_directory(package_name), 'config', 'trajectory_controller.yaml'
    )

    # run the ros2_control_node to handle controller spawning and loading
    # remap topic to /robot_description
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[("/controller_manager/robot_description", "robot_description")],
        output="screen",
    )

    delay_controller_manager = TimerAction(
        period=3.0, 
        actions={controller_manager}
    )

    # === Spawn velocity controller (if cmd_mode is 'velocity') ===
    vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_trajectory_controller"],
        output="screen",
        condition=LaunchConfigurationEquals("cmd_mode", "speed")
    )

    # === Spawn position controller (if cmd_mode is 'position') ===
    pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller"],
        output="screen",
        condition=LaunchConfigurationEquals("cmd_mode", "position")
    )

    # === Spawn joint state broadcaster ===
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
        output="screen",
    )

    # === Register event handlers ===
    timed_vel_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[vel_controller_spawner],
        )
    )

    timed_pos_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[pos_controller_spawner],
        )
    )

    timed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_mode',
            default_value='speed',
            description='choose between position and speed control'
        ),

        DeclareLaunchArgument(
            'real_time',
            default_value='true',
            description='choose between real hardware and mock hardware'
        ),

        rsp,
        delay_controller_manager,
        timed_vel_controller_spawner,
        timed_pos_controller_spawner,
        timed_joint_broad_spawner
    ])