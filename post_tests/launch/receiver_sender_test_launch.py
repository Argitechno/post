from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    receiver_node = Node(
        package='post_stations',
        executable='receiver_station',
        name='receiver_test',
        output='screen',
        arguments=[
            '--loss_mode', 'lossless',
        ]
    )

    sender_node = Node(
        package='post_stations',
        executable='sender_station',
        name='sender_test',
        output='screen',
        arguments=[
            '--destinations', 'receiver_test',
            '--mode', 'round_robin',
            '--send_interval_sec', '0.5',
            '--max_parcels', '3',
            '--loss_mode', 'lossless',
        ]
    )

    return LaunchDescription([
        receiver_node,
        sender_node
    ])
