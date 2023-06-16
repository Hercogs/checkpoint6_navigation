import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    cartographer_config_name = 'cartographer.lua'

    cartographer_node = Node(package='cartographer_ros',
                            executable='cartographer_node',
                            name='cartographer_node',
                            output='screen',
                            parameters=[{'use_sim_time': True}],
                            arguments=['-configuration_directory', cartographer_config_dir,
                                        '-configuration_basename', cartographer_config_name])

    occupancy_grid_node = Node(package='cartographer_ros',
                                executable='occupancy_grid_node',
                                name='occupancy_grid_node',
                                output='screen',
                                parameters=[{'use_sim_time': True}],
                                arguments=['-resolution', '0.05',
                                            '-publish_period_sec', '1.0']
                                )
    
    rviz_config = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'rviz_map_generate_config.rviz')
    rviz_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config]
                    )

    return LaunchDescription([  cartographer_node,
                                occupancy_grid_node,
                                rviz_node])