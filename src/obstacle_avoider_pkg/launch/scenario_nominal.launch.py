#!/usr/bin/env python3
"""
scenario_nominal.launch.py
---------------------------
Scenario nominal : le robot avance dans le couloir avec
les parametres par defaut et evite les obstacles detectes.
Distance de securite = 0.5 m, vitesse = 0.18 m/s.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_avoider_pkg')

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation_complete.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([simulation_launch])
