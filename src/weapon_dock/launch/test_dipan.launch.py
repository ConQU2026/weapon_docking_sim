from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    weapon_dock_pkg = FindPackageShare('weapon_dock')
    joy_pkg = FindPackageShare('joy')
    auto_serial_bridge_pkg = FindPackageShare('auto_serial_bridge')

    serial_config_path = PathJoinSubstitution(
        [auto_serial_bridge_pkg, 'config', 'serial_data.yaml'])

    # js_convert_node = Node(
    #     package='weapon_dock',
    #     executable='js_convert_node',
    #     name='js_convert_node',
    #     output='screen',
    # )
    
    container = ComposableNodeContainer(
            name='serial_pkg_container',
            namespace= '',
            package='rclcpp_components',
            executable='component_container', 
            arguments=['--ros-args', '--log-level', 'debug'],
            composable_node_descriptions=[
                ComposableNode(
                    package="auto_serial_bridge",
                    plugin='auto_serial_bridge::SerialController',
                    name='serial_controller',
                    parameters=[serial_config_path],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                # 你可以在这里继续添加其他组件，让它们跑在同一个进程里
                
                # ComposableNode(
                #     package='joy',
                #     plugin='joy::Joy',
                #     name='joy_node'
                # ),
            ],
            output='screen',
        ) 
    

    ld  = LaunchDescription()
    
    # ld.add_action(js_convert_node)
    ld.add_action(container)
    

    return ld