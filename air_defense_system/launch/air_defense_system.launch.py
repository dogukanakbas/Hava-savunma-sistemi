from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable RViz visualization'
        ),
        
        # Aircraft Controller
        Node(
            package='air_defense_system',
            executable='aircraft_controller.py',
            name='aircraft_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # RRT Path Planner
        Node(
            package='air_defense_system',
            executable='rrt_path_planner.py',
            name='rrt_path_planner',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/air_defense.rviz'],
            condition=LaunchConfiguration('enable_visualization')
        )
    ])
