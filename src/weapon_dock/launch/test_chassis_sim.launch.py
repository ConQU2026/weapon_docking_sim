from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    weapon_dock_pkg = FindPackageShare('weapon_dock')
    r2_pkg = FindPackageShare('r2_description')

    gz_launch_path = PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
    
    world_path = PathJoinSubstitution([weapon_dock_pkg, 'resource', 'worlds', 'robocon2026_world', 'world.sdf'])
    
    xacro_file = PathJoinSubstitution([r2_pkg, 'urdf', 'R2.xacro'])
    robot_description = Command(['xacro ', xacro_file])
    

    ld = LaunchDescription()

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node', 
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    js_convert_node  = Node(
        package='weapon_dock',
        executable='js_convert_node',
        name='js_convert_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            {'use_sim_time': True}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
     
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'R2',
            '-topic', 'robot_description',
            '-x', '3.5',
            '-y', '3.5',
            '-z', '0.1', # Lift it up slightly to ensure no ground collision at spawn
        ],
        output='screen'
    )
    
    # Spawn controllers
    mecanum_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager-timeout', '30'],
        output='screen',
    )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen',
    )


    ld.add_action(joy_node)
    ld.add_action(js_convert_node)

    ld.add_action(AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([weapon_dock_pkg, 'resource', 'worlds'])
    ))


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--verbose',
            'pause': 'false' # Start unpaused to prevent controller timeout
        }.items(),
    ))

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(urdf_spawn_node)
    
    # Add controllers
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node,
            on_exit=[joint_state_broadcaster, mecanum_drive_spawner],
        )
    ))

    return ld
