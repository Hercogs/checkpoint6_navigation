from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Package name
    package_name = 'path_planner_server'

    # RVIZ node
    rviz_config = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz_localization.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_path',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config]
    )


    path_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'planner_server.yaml')
    # Path planner node
    path_planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[path_config_file]
    )
    
    # Controller node
    control_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'controller.yaml')
    controller_node = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                            control_config_file],
            remappings=[
                        ('cmd_vel', 'robot/cmd_vel'),
            ]
    )

    # Navigator coordinator
    bt_navigator_file = os.path.join(get_package_share_directory(package_name), 'config', 'bt_navigator.yaml')
    bt_navigator_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_file]
    )

    # Recovery node
    recovery_file = os.path.join(get_package_share_directory(package_name), 'config', 'recovery.yaml')
    recovery_node = Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_file],
    )




    # Lifecycle node
    lifecycle_node = Node(package='nav2_lifecycle_manager',
                            executable='lifecycle_manager',
                            name='lifecycle_manager_path_planning',
                            output='screen',
                            parameters=[{'use_sim_time': True},
                                        {'autostart': True},
                                        {'node_names': ['planner_server',
                                                        'controller_server',
                                                        'bt_navigator',
                                                        'recoveries_server']}])
                    


    return LaunchDescription([  rviz_node,
                                path_planner_node,
                                controller_node,
                                bt_navigator_node,
                                recovery_node,
                                lifecycle_node]
    )