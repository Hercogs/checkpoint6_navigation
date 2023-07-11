from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Package name
    package_name = 'localization_server'

    # Parmaters
    map_file_name_arg = DeclareLaunchArgument('map_file',
                                            default_value='warehouse_map_sim.yaml',
                                            description='Choose real vs simulated map')
    map_file_name_val = LaunchConfiguration('map_file')
    

    # RVIZ node
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_localization.rviz')
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config]
                    )
    
    # Map server
    map_file_path = PathJoinSubstitution([get_package_share_directory('map_server'), 'config', map_file_name_val])
    map_server_node = Node(package='nav2_map_server',
                            executable='map_server',
                            name='map_server',
                            output='screen',
                            parameters=[{'use_sim_time': False},
                                        {'yaml_filename': map_file_path}])
    
    # Localization
    nav2_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml')
    amcl_node = Node(package='nav2_amcl',
                        executable='amcl',
                        name='amcl',
                        output='screen',
                        parameters=[nav2_yaml
                                    ])


    # Lifecycle node
    lifecycle_node = Node(package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='lifecycle_manager',
                            output='screen',
                            parameters=[{'use_sim_time': False},
                                        {'autostart': True},
                                        {'node_names': ['map_server',
                                                        'amcl']}])
                    


    return LaunchDescription([  rviz_node,
                                map_file_name_arg,
                                map_server_node,
                                amcl_node,
                                lifecycle_node,

                                # LogInfo(condition=LaunchConfigurationEquals('map_file', 'warehouse_map_sim.yaml'),
                                #     msg = "Default map!!"),
    
                                # LogInfo(condition=LaunchConfigurationNotEquals('map_file', 'warehouse_map_sim.yaml'),
                                #     msg = "NOT Default map!!"),

                                ])
                                