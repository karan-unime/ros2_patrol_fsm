from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_path = os.path.join(
        os.environ['HOME'],
        'rosbot_ws',
        'src',
        'sim_launch',
        'worlds',
        'factory_patrol.world'
    )

    return LaunchDescription([
        # Start Gazebo with custom world
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so'
            ],
            output='screen'
        ),

        # Spawn TurtleBot3 Burger in the world at position (0, 0, 0.1)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'tb3',
                '-file', os.path.join(
                    '/opt/ros/humble/share/turtlebot3_gazebo/models',
                    'turtlebot3_burger',
                    'model.sdf'
                ),
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
    ])

