#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('diffbot_bringup'),
                    'urdf',
                    'diffbot.urdf.xacro'
                ]
            ),
        ]
    )

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('diffbot_bringup'),
            'config',
            'controller_manager.yaml',
        ]
    )


    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': urdf_file},
            controller_manager_config
        ],
        remappings=[
            ('~/cmd_vel_unstamped', 'cmd_vel'),
            ('~/odom', 'odom')
        ],
        output="screen"
        )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
        output='screen',
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffbot_control_system', '-c', '/controller_manager'],
        output='screen',
    )

    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = \
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)







    
