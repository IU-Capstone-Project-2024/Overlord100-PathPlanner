from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Global Costmap
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='global_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'update_frequency': 1.0,
                'publish_frequency': 0.5,
                'transform_tolerance': 0.2,
                'rolling_window': False,
                'width': 10.0,
                'height': 10.0,
                'resolution': 0.05,
            }],
        ),
        # Local Costmap
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='local_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'global_frame': 'odom',
                'robot_base_frame': 'base_link',
                'update_frequency': 5.0,
                'publish_frequency': 2.0,
                'transform_tolerance': 0.2,
                'rolling_window': True,
                'width': 3.0,
                'height': 3.0,
                'resolution': 0.05,
            }],
        ),
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'planner_plugin': 'GridBased',
            }],
        ),
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'controller_plugin': 'FollowPath',
            }],
        ),
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'global_costmap',
                    'local_costmap',
                    'planner_server',
                    'controller_server'
                ]
            }]
        ),
    ])
