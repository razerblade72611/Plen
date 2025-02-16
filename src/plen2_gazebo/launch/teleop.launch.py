import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    plen2_description_dir = get_package_share_directory('plen2_gazebo')
    
    # Robot description file
    default_model_path = os.path.join(plen2_description_dir, 'urdf', 'plen2.urdf.xacro')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time if true'),
        
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command(['xacro ', default_model_path]),
            description='Full path to robot URDF file'),
        
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-file', LaunchConfiguration('robot_description'),
                       '-entity', 'plen2_robot']),
    ])

