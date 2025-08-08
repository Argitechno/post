import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from post_msgs.msg import Parcel
from post_stations.processing_station import ProcessingStation
from post_stations.parcel_utils import dict_to_parcel_msg

def run_node(node):
    rclpy.spin(node)

def send_parcel(node, parcel):
    pub = node.create_publisher(Parcel, f"/stations/{node.name}/incoming_parcels", 10)
    msg = dict_to_parcel_msg(parcel['id'], parcel['owner_id'], parcel['instruction_set_key'], parcel['data'])
    # Give time for publisher to set up
    time.sleep(0.5)
    pub.publish(msg)

def main():
    rclpy.init()
    station = ProcessingStation("test_station")

    # Run node in separate thread
    t = threading.Thread(target=run_node, args=(station,), daemon=True)
    t.start()

    # Compose a test parcel
    test_parcel = {
        "id": str(uuid.uuid4()),
        "owner_id": "tester",
        "instruction_set_key": "relay",  # assumes this instruction set exists
        "data": {"message": "hello"}
    }

    send_parcel(station, test_parcel)

    # Allow time for processing
    time.sleep(2)

    # Access station’s parcel queue or any received data (you might want to add)
    # For now, just log success and shutdown

    print("Test complete")

    station.destroy_node()
    rclpy.shutdown()
    t.join(timeout=1)

if __name__ == "__main__":
    main()
