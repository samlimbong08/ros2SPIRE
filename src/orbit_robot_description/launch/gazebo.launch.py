import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    pkg_share = get_package_share_directory('orbit_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'fairino10_v6.urdf')

    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_desc = doc.toxml()

    params = {
        'robot_description': robot_desc,
        'use_sim_time': use_sim_time
    }

    # ---------------------------
    # Robot State Publisher
    # ---------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # ---------------------------
    # Gazebo
    # ---------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'verbose': 'false'}.items()
    )

    # ---------------------------
    # Spawn entity — pass URDF string directly, no topic dependency
    # ---------------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'fairino10_v6_robot',
            '-topic', 'robot_description',
            '-timeout', '120'
        ]
    )

    # ---------------------------
    # Load controllers via CLI — bypasses spawner service discovery bug
    # ---------------------------
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_trajectory_controller'
        ],
        output='screen'
    )

    # ---------------------------
    # Sequencing
    # ---------------------------
    start_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    start_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
    ])