from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    station_names = ['proc1', 'proc2']

    # Processing station nodes
    stations = [
        Node(
            package='post_stations',
            executable='processing_station',
            name=name,
            output='screen',
            arguments=[name],
        )
        for name in station_names
    ]

    # Publisher test node: runs a Python script that sends the parcel once stations are up
    publisher_node = Node(
        package='post_tests',
        executable='dynamic_loop_publisher',  # See step 2 below
        name='dynamic_loop_publisher',
        output='screen',
    )

    return LaunchDescription(stations + [publisher_node])
