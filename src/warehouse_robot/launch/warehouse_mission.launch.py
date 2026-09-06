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
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(turtlebot3_gazebo_share, 'worlds', 'turtlebot3_world.world'),
        }.items()
    )
    turtlebot3_description_share = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(turtlebot3_description_share, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read().replace('${namespace}', '')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
        output='screen'
    )

    spawn_tb3 = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'burger',
                    '-file', os.path.join(turtlebot3_gazebo_share, 'models', 'turtlebot3_burger', 'model.sdf'),
                    '-x', '0.0', '-y', '0.0', '-z', '0.01',
                    '-timeout', '60'
                ],
                output='screen'
            )
        ]
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'params_file': nav2_params_path,
            'use_sim_time': use_sim_time,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    replan_monitor_node = Node(
        package=pkg_name, executable='replan_monitor', name='replan_monitor',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    obstacle_spawner_node = Node(
        package=pkg_name, executable='obstacle_spawner', name='obstacle_spawner',
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    # mission_controller runs separately, in its own terminal — not launched here

    # 5. Initial pose — wait for /odom to actually have a live publisher
    # (i.e. the robot has really spawned in Gazebo), not just for AMCL's
    # lifecycle state, which goes active long before the robot exists.
    set_initial_pose = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]"; '
            'do sleep 1; done; sleep 2; '
            "ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
            "\"{header: {frame_id: 'map'}, pose: {pose: {position: "
            "{x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\" --once -w 1"
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_tb3_model,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_tb3,
        nav2_launch,
        rviz_node,
        replan_monitor_node,
        obstacle_spawner_node,
        set_initial_pose,
    ])
