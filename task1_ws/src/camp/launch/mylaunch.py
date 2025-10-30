#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_camp = get_package_share_directory('camp')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    world_arg = DeclareLaunchArgument(
        'world', default_value='task2.world', description='World file in camp/worlds/'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_camp, 'map', 'map.yaml'), description='Full path to map.yaml'
    )
    set_sim_time_env = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')

    world = PathJoinSubstitution([FindPackageShare('camp'), 'worlds', LaunchConfiguration('world')])
    urdf_path = os.path.join(pkg_camp, 'models', 'husky_robot_model', 'model.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true', 'pause': 'false', 'use_sim_time': 'true'}.items()
    )

    rsp = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    output='screen'
)

    joint_state_pub = Node(
        package='camp',
        executable='simple_joint_state_publisher.py',
        name='simple_joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'husky_robot',
            '-x', '-7', 
            '-y', '5', 
            '-z', '0', 
            '-Y', '3.14'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}]
    )

    amcl_config_path = os.path.join(pkg_camp, 'config', 'amcl_config.yaml')

    amcl = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[amcl_config_path, {'use_sim_time': True}]
)

    rviz_config = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        set_sim_time_env,
        world_arg,
        map_yaml_arg,
        gazebo,
        rsp,
        joint_state_pub,
        spawn_entity,
        map_server,
        amcl,
        rviz_node
    ])