#!/usr/bin/env python3
''' 
*****************************************************************************************
*  Filename:          start_world_fortress_launch.py
*  Description:       Launch Ignition Gazebo (Fortress) world via ros_gz_sim
*  Modified by:       Sahil
*  Author:            e-Yantra Team
*****************************************************************************************
'''

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix

pkg_name = 'eyantra_warehouse'

def generate_launch_description():
    # 1) Locate package share & install directories
    pkg_share = get_package_share_directory(pkg_name)
    pkg_prefix = get_package_prefix(pkg_name)

    # 2) Declare the world file launch argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_share, 'worlds', 'eyantra_warehouse_task0.world'), ''],
        description='Ignition Fortress world file'
    )

    # 3) Set environment variables for Fortress resource & plugin paths
    res_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    res_path += f":{pkg_share}/models"
    plg_path = os.environ.get('IGN_GAZEBO_PLUGIN_PATH', '')
    plg_path += f":{pkg_prefix}/lib"

    set_ign_res = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', res_path)
    set_ign_plg = SetEnvironmentVariable('IGN_GAZEBO_PLUGIN_PATH', plg_path)

    # 4) Include the ros_gz_sim launch for Ignition Fortress
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('world')
        }.items()
    )

    # 5) Assemble the launch description
    return LaunchDescription([
        world_arg,
        set_ign_res,
        set_ign_plg,
        gz_sim
    ])
