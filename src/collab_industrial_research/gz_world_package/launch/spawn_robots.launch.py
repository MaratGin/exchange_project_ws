import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    

    gazebo_world_package = get_package_share_directory('gz_world_package')
    ur5e_model_pkg = get_package_share_directory('ur5e_model')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    kuka_model_pkg = get_package_share_directory('kuka_kr3_model')




    ur_description_cmd = Command([
        'xacro ',
        PathJoinSubstitution([ur5e_model_pkg, 'urdf', 'ur5e_test.urdf']), ' name:=ur'
    ])

    kuka_description_cmd = Command([
        'xacro ',
        PathJoinSubstitution([kuka_model_pkg, 'urdf', 'kuka_kr3r540.urdf.xacro']), ' name:=kuka'
    ])

    world_arg = DeclareLaunchArgument(
        'research_world', default_value='research_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    ur5e_arg = DeclareLaunchArgument(
        'ur5e', default_value='ur5e_robot.xacro',
        description='Name of the URDF description to load'
    )
    kuka_arg = DeclareLaunchArgument(
        'kukakr3', default_value='kuka_kr3r540.urdf.xacro',
        description='Name of the URDF description to load'
    )
    print("1")
    # launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)

    # Define the path to your URDF or Xacro file
    ur5e_file_path = PathJoinSubstitution([
        ur5e_model_pkg,  # Replace with your package name
        "urdf",
        LaunchConfiguration('ur5e')  # Replace with your URDF or Xacro file
    ])
    print("2")


    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_world_package, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('research_world'),
        }.items()
    )
    print("3")


    # Start rviz
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(gazebo_world_package, 'rviz', 'rviz.rviz')],
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     parameters=[
    #         {'use_sim_time': True},
    #     ]
    # )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_ur_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur5e",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    spawn_kuka_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "kuka_kr3",
            "-topic", "kuka_robot_description",
            "-x", "10.0", "-y", "10.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    print("4")
    ur_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(ur_description_cmd, value_type=str),
             'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    kuka_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(kuka_description_cmd, value_type=str),
             'use_sim_time': True},
        ],
        remappings=[
            ('/kuka/tf', 'tf'),
            ('/kuka/tf_static', 'tf_static')
        ]
    )
    print("5")

    # gz_bridge_node = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    #         "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
    #         "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
    #         "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"
    #     ],
    #     output="screen",
    #     parameters=[
    #         {'use_sim_time': True},
    #     ]
    # )
    # print("6")

    # trajectory_node = Node(
    #     package='mogi_trajectory_server',
    #     executable='mogi_trajectory_server',
    #     name='mogi_trajectory_server',
    # )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(ur5e_arg)
    launchDescriptionObject.add_action(kuka_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(spawn_ur_urdf_node)
    launchDescriptionObject.add_action(spawn_kuka_urdf_node)
    launchDescriptionObject.add_action(ur_robot_state_publisher_node)
    launchDescriptionObject.add_action(kuka_robot_state_publisher_node)

    # launchDescriptionObject.add_action(gz_bridge_node)
    # launchDescriptionObject.add_action(trajectory_node)




    return launchDescriptionObject