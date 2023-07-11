from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Package name
    package_name = 'map_server'

    # Parmaters
    map_file_name_arg = DeclareLaunchArgument('map_file',
                                            default_value='warehouse_map_sim.yaml',
                                            description='Choose real vs simulated map')
    map_file_name_val = LaunchConfiguration('map_file')


    # RVIZ node
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_map_server.rviz')
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config]
                    )
    
    # Map server
    map_file_path = PathJoinSubstitution([get_package_share_directory(package_name), 'config', map_file_name_val])
    map_server_node = Node(package='nav2_map_server',
                            executable='map_server',
                            name='map_server',
                            output='screen',
                            parameters=[{'use_sim_time': False},
                                        {'yaml_filename': map_file_path}])


    # Lifecycle node
    lifecycle_node = Node(package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='lifecycle_manager',
                            output='screen',
                            parameters=[{'use_sim_time': False},
                                        {'autostart': True},
                                        {'node_names': ['map_server']}])
                    


    return LaunchDescription([  rviz_node,
                                map_file_name_arg,
                                map_server_node,
                                lifecycle_node])