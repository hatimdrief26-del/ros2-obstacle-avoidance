#!/usr/bin/env python3
"""
Fichier launch principal : simulation_complete.launch.py
---------------------------------------------------------
Lance l'ensemble du systeme :
  1. Conversion URDF (xacro -> URDF)
  2. robot_state_publisher
  3. Gazebo avec le monde obstacle_world.sdf
  4. Spawn du robot dans Gazebo
  5. RViz avec la configuration projet
  6. Noeuds ROS 2 : evitement_obstacles_node + moniteur_etat_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('obstacle_avoider_pkg')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ---- Fichiers ----
    urdf_xacro   = os.path.join(pkg_share, 'urdf',   'robot.urdf.xacro')
    world_file   = os.path.join(pkg_share, 'worlds', 'obstacle_world.sdf')
    rviz_config  = os.path.join(pkg_share, 'config', 'robot_view.rviz')

    # ---- Arguments launch ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation Gazebo'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- 1. Conversion XACRO -> URDF (en memoire) ----
    robot_description_cmd = Command(['xacro ', urdf_xacro])

    # ---- 2. robot_state_publisher ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_cmd,
            'use_sim_time': use_sim_time,
        }]
    )

    # ---- 3. Gazebo (serveur + client) ----
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # ---- 4. Spawn du robot dans Gazebo ----
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'obstacle_avoider_robot',
            '-topic',  'robot_description',
            '-x', '-3.5',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # ---- 5. RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---- 6. Noeuds applicatifs (avec delai pour laisser Gazebo demarrer) ----
    evitement_node = Node(
        package='obstacle_avoider_pkg',
        executable='evitement_obstacles_node',
        name='evitement_obstacles_node',
        output='screen',
        parameters=[{
            'use_sim_time':           use_sim_time,
            'distance_securite':      0.5,
            'distance_arret_urgence': 0.20,
            'vitesse_lineaire':       0.18,
            'vitesse_angulaire':      0.6,
            'angle_detection_frontal': 45.0,
        }]
    )

    moniteur_node = Node(
        package='obstacle_avoider_pkg',
        executable='moniteur_etat_node',
        name='moniteur_etat_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'seuil_alerte': 0.5,
        }]
    )

    # Delai pour le spawn et les noeuds applicatifs
    spawn_delayed  = TimerAction(period=4.0,  actions=[spawn_robot])
    nodes_delayed  = TimerAction(period=6.0,  actions=[evitement_node, moniteur_node])
    rviz_delayed   = TimerAction(period=5.0,  actions=[rviz_node])

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_delayed,
        rviz_delayed,
        nodes_delayed,
    ])
