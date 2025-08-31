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
        
        # Hedef Takip Kontrolcüsü
        Node(
            package='target_tracking_system',
            executable='target_tracking_controller.py',
            name='target_tracking_controller',
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
            arguments=['-d', 'config/target_tracking.rviz'],
            condition=LaunchConfiguration('enable_visualization')
        )
    ])
