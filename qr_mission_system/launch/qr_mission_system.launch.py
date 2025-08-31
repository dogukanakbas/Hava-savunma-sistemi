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
        
        # QR Dalış Kontrolcüsü
        Node(
            package='qr_mission_system',
            executable='qr_dive_controller.py',
            name='qr_dive_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # QR Pozisyon Yayınlayıcısı
        Node(
            package='qr_mission_system',
            executable='qr_position_publisher.py',
            name='qr_position_publisher',
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
            arguments=['-d', 'config/qr_mission.rviz'],
            condition=LaunchConfiguration('enable_visualization')
        )
    ])
