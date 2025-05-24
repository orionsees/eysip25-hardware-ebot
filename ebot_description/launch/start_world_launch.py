#!/usr/bin/env python3
''' 
*****************************************************************************************
*  Filename:			start_world_launch.py
*  Description:         This file launches the world in Gazebo. 
*  Modified by:         Sahil
*  Author:				e-Yantra Team
*****************************************************************************************
'''

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name = 'eyantra_warehouse'

def generate_launch_description():
    # Where your models & worlds live
    pkg_models_dir = get_package_share_directory(pkg_name)
    install_dir    = get_package_prefix(pkg_name)

    # 1) Export Ignition resource & plugin paths.
    ignRES = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ignPLG = os.environ.get('IGN_GAZEBO_PLUGIN_PATH',   '')

    # Add your models folder
    ignRES = f"{ignRES}:{pkg_models_dir}/models"
    # Add your plugin folder (where your .so lives)
    ignPLG = f"{ignPLG}:{install_dir}/lib"

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = ignRES
    os.environ['IGN_GAZEBO_PLUGIN_PATH']   = ignPLG

    # 2) Declare the world file arg (same default as before).
    world_arg = DeclareLaunchArgument('world',default_value=[os.path.join(pkg_models_dir, 'worlds','eyantra_warehouse_task0.world'),''],description='SDF world file')

    # 3) Include the ros_gz_sim launch for ignition.
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # pass your world path directly to gz sim
            'gz_args': LaunchConfiguration('world')
        }.items()
    )

    return LaunchDescription([world_arg,gz_sim,])
