#!/usr/bin/env python3
"""
scenario_perturbe.launch.py
-----------------------------
Scenario perturbe : la distance de securite est reduite a 0.25 m
et la vitesse est augmentee a 0.25 m/s, ce qui oblige le robot
a reagir beaucoup plus tard. On observe le comportement de securite
(arret d urgence) quand la distance de detection est trop courte.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_avoider_pkg')

    urdf_xacro  = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file  = os.path.join(pkg_share, 'worlds', 'obstacle_world.sdf')
    rviz_config = os.path.join(pkg_share, 'config', 'robot_view.rviz')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    robot_description_cmd = Command(['xacro ', urdf_xacro])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_cmd,
            'use_sim_time': True,
        }]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'obstacle_avoider_robot',
            '-topic', 'robot_description',
            '-x', '-3.5', '-y', '0.0', '-z', '0.05', '-Y', '0.0',
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Scenario perturbe : distance securite reduite, vitesse elevee
    evitement_node = Node(
        package='obstacle_avoider_pkg',
        executable='evitement_obstacles_node',
        name='evitement_obstacles_node',
        output='screen',
        parameters=[{
            'use_sim_time':           True,
            'distance_securite':      0.25,   # Distance de securite reduite
            'distance_arret_urgence': 0.12,   # Arret d urgence tres proche
            'vitesse_lineaire':       0.25,   # Vitesse elevee
            'vitesse_angulaire':      0.8,
            'angle_detection_frontal': 45.0,
        }]
    )

    moniteur_node = Node(
        package='obstacle_avoider_pkg',
        executable='moniteur_etat_node',
        name='moniteur_etat_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'seuil_alerte': 0.25}]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        TimerAction(period=4.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[rviz_node]),
        TimerAction(period=6.0, actions=[evitement_node, moniteur_node]),
    ])
