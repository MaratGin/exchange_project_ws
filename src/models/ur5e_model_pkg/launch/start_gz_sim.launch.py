from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import os



def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    pkg_path = FindPackageShare('ur5e_model_pkg')  # Replace with your own package name
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    world_name = LaunchConfiguration("world_name")
    bridge_file_path = LaunchConfiguration('bridge_file_path')

    return LaunchDescription([
        DeclareLaunchArgument(
		name='bridge_file_path',
		default_value=os.path.join(get_package_share_directory('ur5e_model_pkg'), 'config', 'bridge.yaml'),
		description='Bridge configuration'
	),
        DeclareLaunchArgument(
            "world_name",
            description="GZ SIM world name.",
            default_value="start_world.sdf",
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([pkg_path, 'models'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution([pkg_path, 'worlds', world_name]),  # Replace with your own world file
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
            remappings=[('/example_imu_topic',
                         '/remapped_imu_topic'),],
            output='screen'
        ),
        Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		output='screen',
		parameters=[{'config_file': bridge_file_path}]
	    ),
    ])