#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:           ebot_gazebo_fortress_launch.py
*  Description:        Launch Ignition Gazebo Fortress world and spawn the ebot.
*  Modified by:        Sahil
*  Author:             e-Yantra Team
*****************************************************************************************
'''

import os
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Package details
    pkg_name = 'ebot_description'
    pkg_ebot = get_package_share_directory(pkg_name)
    pkg_ebot_share = get_package_share_directory(pkg_name)
    pkg_ebot_prefix = get_package_prefix(pkg_name)
    pkg_world = get_package_share_directory('eyantra_warehouse')

    # 1) Declare use_sim_time arg
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock'
    )

    # 2) Set Ignition resource & plugin paths for Fortress
    ign_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ign_res += f":{pkg_ebot_share}/models"
    ign_plg = os.environ.get('IGN_GAZEBO_PLUGIN_PATH', '')
    ign_plg += f":{pkg_ebot_prefix}/lib"
    set_ign_res = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_res)
    set_ign_plg = SetEnvironmentVariable('IGN_GAZEBO_PLUGIN_PATH', ign_plg)

    # 3) Include the Fortress world launch (from eyantra_warehouse)
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ebot, 'launch', 'start_world_launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_world,
            'worlds',
            'eyantra_warehouse_task0.world'
        ])}.items(),
    )

    # 4) Process the robot_description xacro
    xacro_file = os.path.join(pkg_ebot_share, 'models', 'ebot', 'ebot_description.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 5) Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='ebot_state_publisher', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'robot_description': robot_desc}]
    )

    # 6) Static transform world->odom
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_transform_publisher',
        output='screen',
        arguments=[ '1.6', '-2.4', '-0.8', '3.14', '0', '0', 'world', 'odom' ],
        parameters=[ {'use_sim_time': LaunchConfiguration('use_sim_time')} ]
    )

    # 7) Spawn the ebot after a short delay
    spawn_ebot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim', executable='create', name='spawn_ebot',
                output='screen',
                arguments=['-name', 'ebot', '-topic', 'robot_description',
                           '-x', '1.84', '-y', '-9.05', '-z', '0.1', '-Y', '3.14'],
            )
        ]
    )

    # 8) Bridge /clock topic correctly
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ebot, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Assemble launch description
    return LaunchDescription([
        use_sim_time_arg,
        set_ign_res,
        set_ign_plg,
        start_world,
        rsp_node,
        static_tf,
        spawn_ebot,
        bridge
    ])
