import time
import uuid

import rclpy
from rclpy.node import Node
from post_msgs.msg import Parcel
from post_stations.parcel_utils import dict_to_parcel_msg

def main(args=None):
    rclpy.init(args=args)
    node = Node('dynamic_loop_publisher')

    station_names = ['proc1', 'proc2']
    ttl = 2 * len(station_names)

    parcel_data = {
        "ttl": str(ttl),
        "destinations": ",".join(station_names),
        "destination_index": "1",
        "payload": "test_payload",
    }

    parcel = {
        "id": str(uuid.uuid4()),
        "owner_id": "tester",
        "instruction_set_key": "dynamic_loop",
        "data": parcel_data,
    }

    publisher = node.create_publisher(Parcel, f"/stations/{station_names[0]}/incoming_parcels", 10)

    # Sleep briefly to allow publisher to set up and stations to spin up
    time.sleep(1)

    msg = dict_to_parcel_msg(parcel['id'], parcel['owner_id'], parcel['instruction_set_key'], parcel['data'])
    publisher.publish(msg)
    node.get_logger().info(f"Sent parcel {parcel['id']} with ttl={ttl} to {station_names[0]}")

    # Keep node alive a bit to ensure send completes
    time.sleep(2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
