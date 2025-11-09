from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration('namespace')
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "simulation_controllers:=",
            controllers_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    my_pkg = get_package_share_directory('ur5e_model_pkg')
    gz_sim_launch_path = os.path.join(my_pkg, 'launch', 'start_gz_sim.launch.py')


    # Running another launch file start_gz_sim.launch.py
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            'world_name': 'start_world.sdf',
        }.items()
    )

    joint_controllers_file = os.path.join(
        get_package_share_directory('ur5e_model_pkg'), 'config', 'ur5_controllers.yaml'
    )


    declare_bridge_file = DeclareLaunchArgument(
		name='bridge_file_path',
		default_value=os.path.join(get_package_share_directory('ur5e_model_pkg'), 'config', 'bridge.yaml'),
		description='Bridge configuration'
	)

    declare_controllers_file = DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur5e_model_pkg"), "config", "ur5e_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )


    declare_tf_prefixes = DeclareLaunchArgument(
            "tf_prefix",
            default_value='ur5e_',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )

    declate_ur_type = DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5e",
        )

    declare_safety_pos_margin =  DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    declare_safety_k_position = DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )

    declare_discription_file = DeclareLaunchArgument(
            name="description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur5e_model_pkg"), "urdf", "ur5e_camera.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    
    declare_safety_limits = DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )

    declare_namespace_cmd = DeclareLaunchArgument(
		name='namespace',
		default_value='ur5e',
		description='Top-level namespace'
	)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='true',
		description='Use simulation (Gazebo) clock if true'
	)

    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')


    # spawn the robot
    print("SPAWN THE ROBOT")
    spawn_the_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic','/robot_description',
            '-name', 'ur',
            '-x', '0.5',  # Position at table center
            '-y', '0.0',
            '-z', '0.4',  # Position at table height
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # [namespace, '/robot_description']

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        joint_controllers_file,
        {'robot_description': LaunchConfiguration('description_file')}
    ],)


    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # arm_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    # delay_joint_state_broadcaster = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=controller_manager_node,
    #         on_start=[joint_state_broadcaster_spawner],
    #     )
    # )

    # delay_arm_controller = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_start=[arm_trajectory_controller_spawner],
    #     )
    # )



        		#rema#ppings=[('/tf','tf'),('/tf_static','tf_static')],      remappings=[('/robot_description', [namespace, '/robot_description'])]

    # Launch Description
    ld.add_action(declare_controllers_file)
    ld.add_action(declare_tf_prefixes)
    ld.add_action(declate_ur_type)
    ld.add_action(declare_safety_pos_margin)
    ld.add_action(declare_safety_k_position)
    ld.add_action(declare_safety_limits)
    ld.add_action(declare_discription_file)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gz_sim)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)

    ld.add_action(spawn_the_robot)
    ld.add_action(rsp_node)
    ld.add_action(controller_manager_node)
    # ld.add_action(delay_joint_state_broadcaster)
    # ld.add_action(delay_arm_controller)

    return ld