from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    get_pose_client_node = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        parameters=[{'debug_topics': True},]
    )

    basic_grasping_perception_node = Node(
        package='get_pose_perception',
        executable='get_pose_client',
        name='get_pose_client',
        output='screen',
        parameters=[])

    return LaunchDescription(
        [
        get_pose_client_node,
        basic_grasping_perception_node
        ]
    )

