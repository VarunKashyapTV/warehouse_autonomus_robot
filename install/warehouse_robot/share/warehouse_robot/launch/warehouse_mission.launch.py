import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = 'warehouse_robot'

    pkg_share = get_package_share_directory(pkg_name)
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    nav2_params_path = os.path.join(nav2_bringup_share, 'params', 'nav2_params.yaml')
    rviz_config_path = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # 1. Gazebo — TurtleBot3 world setup
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.01',  
        }.items()
    )

    # 2. Nav2 bringup — Enable autostart and pass sim time explicitly
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': nav2_params_path,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items()
    )

    # 3. RViz2 setup
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 4. Custom Package Nodes
    replan_monitor_node = Node(
        package=pkg_name, executable='replan_monitor', name='replan_monitor',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    obstacle_spawner_node = Node(
        package=pkg_name, executable='obstacle_spawner', name='obstacle_spawner',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    mission_controller_node = Node(
        package=pkg_name, executable='mission_controller', name='mission_controller',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. Publish initial pose to lock AMCL map -> odom transformation
    set_initial_pose = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    "{header: {frame_id: 'map'}, pose: {pose: {position: "
                    "{x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}",
                    '--once',
                ],
                output='screen'
            )
        ]
    )

    # 6. Delayed start for monitoring, spawner, and mission controller
    # Ensures Gazebo simulation clocks and Nav2 servers are fully active first
    delayed_nodes = TimerAction(
        period=12.0,
        actions=[
            replan_monitor_node,
            obstacle_spawner_node,
            mission_controller_node
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        set_tb3_model,
        gazebo_launch,
        nav2_launch,
        rviz_node,
        set_initial_pose,
        delayed_nodes,
    ])