from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')
    weapon_dock_pkg = FindPackageShare('weapon_dock')
    joy_pkg = FindPackageShare('joy')

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
    world_path = PathJoinSubstitution([weapon_dock_pkg, 'resource', 'worlds', 'robocon2026_world', 'world.sdf'])
    
    xacro_file = PathJoinSubstitution([weapon_dock_pkg, 'urdf', 'robot.urdf.xacro'])
    robot_description = Command(['xacro',' ', xacro_file])
    

    ld = LaunchDescription()

    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    
    
    ld.add_action(Node(
        package='weapon_dock',
        executable='js_convert_node',
        name='js_convert_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    
    
    ld.add_action(AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([weapon_dock_pkg])
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [world_path], 
            'on_exit_shutdown': 'True'
        }.items(),
    ))

    # 发布 robot_description
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}  
        ]
    ))

    # 在 Gazebo 中生成模型 (要运行一次就退出，不需要 sim_time)
    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_robot', '-z', '0.5', '-x', '4', '-y', '4'],
        output='screen'
    ))

    # 桥接节点
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/d435/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True} 
        ]
    ))

    return ld