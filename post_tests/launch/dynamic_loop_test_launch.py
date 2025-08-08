from launch import LaunchDescription
from launch_ros.actions import Node
import json

def generate_launch_description():
    station_names = ['proc1', 'proc2']

    # Processing station nodes
    stations = [
        Node(
            package='post_stations',
            executable='processing_station',
            name=name,
            output='screen',
            arguments=[
                '--loss_mode', 'lossy',
                '--depth', '10'
            ]
        )
        for name in station_names
    ]
    
    stations.append(
        Node(
            package='post_stations',
            executable='sender_station',
            name='dynamic_loop_sender',
            output='screen',
            arguments=[
                '--loss_mode', 'lossy',
                '--depth', '10',
                '--destinations', 'proc1',
                '--mode', 'broadcast_once',
                '--max_parcels', '1',
                '--send_interval_sec', '1.0'
            ],
            parameters=[{
                'instruction_set_key': 'dynamic_loop',
                'parcel_data_keys': ['ttl', 'destinations', 'destination_index'],
                'parcel_data_vals': ['4', 'proc1,proc2', '1']
            }]

        )
    )

    return LaunchDescription(stations)
