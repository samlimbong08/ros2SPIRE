import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the package share directory
    pkg_share = get_package_share_directory('orbit_robot_description')
    
    # 2. Define default paths for your URDF and RViz config
    # (Using 'robot.urdf' as a placeholder—make sure it matches your actual file name)
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'fairino10_v6.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # 3. Declare launch arguments
    urdf_model = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_path,
        description='Absolute path to robot urdf file'
    )
    
    rviz_config = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    # This argument allows you to toggle the manual GUI sliders on/off from the terminal
    gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui sliders'
    )

    # 4. Read the URDF file contents into a string variable
    with open(default_urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 5. Define the nodes
    
    # Robot State Publisher reads the URDF and broadcasts the 3D transforms (TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Joint State Publisher GUI will ONLY launch if use_gui is set to 'true'
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    # RViz2 Node opens the visualization window
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # 6. Return the LaunchDescription containing all configurations and nodes
    return LaunchDescription([
        urdf_model,
        rviz_config,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])