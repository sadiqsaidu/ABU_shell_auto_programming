from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shell_simulation', # Your package name
            executable='perception_node', # The entry point in setup.py
            name='perception_node_instance', # ROS Name for this specific node instance
            output='screen'
        ),
        Node(
            package='shell_simulation',
            executable='planning_node',
            name='planning_node_instance',
            output='screen'
        ),
        Node(
            package='shell_simulation',
            executable='control_node',
            name='control_node_instance',
            output='screen'
        ),
    ])