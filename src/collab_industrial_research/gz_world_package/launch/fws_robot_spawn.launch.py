import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    # Packages
    ur5e_model_pkg = FindPackageShare('ur5e_model')
    gazebo_pkg = FindPackageShare('gz_world_package')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    # Paths
    ur5e_model_path = PathJoinSubstitution([ur5e_model_pkg, 'urdf', 'ur5e_model.urdf'])
    world_path = PathJoinSubstitution([gazebo_pkg, 'worlds', 'research_world.sdf'])
    print("1231321313123123")

    # fws_robot_description_path = os.path.join(
    #     get_package_share_directory('fws_robot_description'))
    
    # research_sim_path = os.path.join(
    #     get_package_share_directory('gz_world_package'))

    # Set gazebo sim resource path
    # gazebo_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=[
    #         os.path.join(research_sim_path, 'worlds'), ':' +
    #         str(Path(fws_robot_description_path).parent.resolve())
    #         ]
    #     )

    # arguments = LaunchDescription([
    #             DeclareLaunchArgument('world', default_value='research_world',
    #                       description='Gz sim World'),
    #        ]
    # )




    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='SDF world file'
    )

    # Start Gazebo server + client
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_server.launch.py')
        ),
        launch_arguments={
                'gz_args': PathJoinSubstitution([gazebo_pkg, 'worlds/research_world.sdf']),  # Replace with your own world file
                'on_exit_shutdown': 'True'
            }.items(),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # gz_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
    #     ),
    #     launch_arguments={'gz_args': f'-g {world_path}'}.items()
    # )

    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
    #             launch_arguments=[
    #                 ('gz_args', [LaunchConfiguration('research_world'),
    #                              '.sdf',
    #                              ' -v 4',
    #                              ' -r']
    #                 )
    #             ]
    #          )

    # xacro_file = os.path.join(fws_robot_description_path,
    #                           'robots',
    #                           'fws_robot.urdf.xacro')

    # doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    # robot_desc = doc.toprettyxml(indent='  ')

    # params = {'robot_description': robot_desc}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[ur5e_model_path]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', "robot_description",
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'ur5e_robot'],
    )

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_forward_velocity_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'forward_velocity_controller'],
    #     output='screen'
    # )

    # load_forward_position_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'forward_position_controller'],
    #     output='screen'
    # )

    # # Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
    #     output='screen'
    # )

    # rviz_config_file = os.path.join(ur5e_model_path, 'config', 'fws_robot_config.rviz')

    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

    # return LaunchDescription([
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=gz_spawn_entity,
    #             on_exit=[load_joint_state_controller],
    #         )
    #     ),
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #            target_action=load_joint_state_controller,
    #            on_exit=[load_forward_velocity_controller,
    #                     load_forward_position_controller],
    #         )
    #     ),
    #     gazebo_resource_path,
    #     arguments,
    #     gazebo,
    #     node_robot_state_publisher,
    #     gz_spawn_entity,
    #     bridge,
    #     rviz,
    # ])

    return LaunchDescription([
        # gz_server,
        # gz_client,
        start_gazebo_server,
        robot_state_publisher,
        gz_spawn_entity,
    ])
