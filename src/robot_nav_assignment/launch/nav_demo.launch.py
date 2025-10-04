from launch import LaunchDescription
from launch_ros.actions import Node
import json

def generate_launch_description():

    # Define waypoints as a JSON string
    waypoints = json.dumps([
        {'x': 1.0, 'y': 2.0},
        {'x': 3.0, 'y': 4.0},
        {'x': 5.0, 'y': 2.5}
    ])

    return LaunchDescription([
        Node(
            package='robot_nav_assignment',
            executable='simulator',
            name='diff_drive_simulator',
            output='screen'
        ),
        Node(
            package='robot_nav_assignment',
            executable='path_server',
            name='path_server',
            output='screen',
            parameters=[{'waypoints': waypoints}]
        ),
        Node(
            package='robot_nav_assignment',
            executable='controller',
            name='pp_controller',
            output='screen',
            parameters=[{'waypoints': waypoints}]
        ),
    ])
