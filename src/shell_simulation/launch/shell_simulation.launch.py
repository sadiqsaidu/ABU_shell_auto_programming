from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shell_simulation', 
            executable='perception_node', 
            name='perception_node_instance', 
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