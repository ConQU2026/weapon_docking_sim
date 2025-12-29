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


    js_convert_node = Node(
        package='weapon_dock',
        executable='js_convert_node',
        name='js_convert_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
    )
    
    container = ComposableNodeContainer(
                name='serial_pkg_container',
                namespace= '',
                package='rclcpp_components',
                executable='component_container', 


                arguments=['--ros-args', '--log-level', 'serial_controller:=debug'],
                
                composable_node_descriptions=[
                    ComposableNode(
                        package="auto_serial_bridge",   
                        plugin='auto_serial_bridge::SerialController',
                        name='serial_controller', 
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                    
                    ComposableNode(
                        package='joy',
                        plugin='joy::Joy',
                        name='joy_node',
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ],
                output='screen',
            )
        

    ld  = LaunchDescription()
    
    ld.add_action(js_convert_node)
    ld.add_action(container)
    

    return ld