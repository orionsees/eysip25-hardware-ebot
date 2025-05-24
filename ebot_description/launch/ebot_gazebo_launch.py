#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:			ebot_gazebo_launch.py
*  Description:         This file launches the world and spawns the ebot in the world.
*  Modified by:         Sahil
*  Author:				e-Yantra Team
*****************************************************************************************
'''

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    pkg_name   = 'ebot_description'
    pkg_share  = get_package_share_directory(pkg_name)
    pkg_prefix = get_package_prefix(pkg_name)

    # 1) Setup Ignition resource & plugin paths.
    ign_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_plg = os.environ.get('IGN_GAZEBO_PLUGIN_PATH',   '')

    # Add your package’s worlds/models and any plugins
    ign_res += f":{pkg_share}/models"
    ign_plg += f":{pkg_prefix}/lib"

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = ign_res
    os.environ['IGN_GAZEBO_PLUGIN_PATH']   = ign_plg

    # 2) Prepare robot_description from XACRO.
    xacro_file = os.path.join(pkg_share, 'models', 'ebot', 'ebot_description.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 2) Launch the eyantra_world from eyantra_warehouse package.
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ebot_description'), 'launch', 'start_world_launch.py'),
        )
    )
    
    # 3) Launch Ignition Gazebo with default world.
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r'],  # server-only, run immediately
        output='screen'
    )

    # 4) Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ebot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 5) world→odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['1.6', '-2.4', '-0.8', '3.14', '0', '0', 'world', 'odom']
    )

    # 6) Spawn the ebot into Gazebo Ignition.
    spawn_ebot = TimerAction(period=2.0,
        actions=[ Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_ebot',
            output='screen',
            arguments=['-name',  'ebot','-topic', 'robot_description',
                '-x', '1.84', '-y', '-9.05', '-z', '0.1', '-Y', '3.14']
        )]
    )

    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='True',
                               description='Use simulation time'),
        start_world,
        gz_sim,
        rsp_node,
        static_tf,
        spawn_ebot,
    ])
