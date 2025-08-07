import rclpy
from rclpy.node import Node
from post_stations.sender_station import SenderStation
from post_stations.receiver_station import ReceiverStation
from post_stations.parcel_utils import parcel_msg_to_dict
import threading
import time

class TestReceiverStation(ReceiverStation):
    def __init__(self, name):
        super().__init__(name)
        self.received_parcels = []
    
    def _parcel_callback(self, msg):
        parcel = parcel_msg_to_dict(msg)
        self.get_logger().info(f"[{self.name}] Test received parcel: {parcel['id']}")
        self.received_parcels.append(parcel)
        self.parcel_queue.put(parcel)

def main():
    rclpy.init()

    sender_name = "sender_test"
    receiver_name = "receiver_test"
    
    # Receiver subscribes on its topic
    receiver = TestReceiverStation(receiver_name)
    # Sender sends only to receiver_name with max_parcels=3 and fast interval
    sender = SenderStation(
        sender_name,
        destinations=[receiver_name],
        mode="round_robin",
        send_interval_sec=0.5,
        max_parcels=3,
    )
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sender)
    executor.add_node(receiver)
    
    # Run executor in a separate thread so we can timeout
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Wait max 5 seconds for parcels to be received
    timeout_sec = 5.0
    start_time = time.time()
    while len(receiver.received_parcels) < 3 and (time.time() - start_time) < timeout_sec:
        time.sleep(0.1)

    # Shutdown ROS
    executor.shutdown()
    sender.destroy_node()
    receiver.destroy_node()
    rclpy.shutdown()

    # Check test result
    if len(receiver.received_parcels) == 3:
        print("TEST PASSED: Receiver got all parcels.")
        return 0
    else:
        print(f"TEST FAILED: Expected 3 parcels, got {len(receiver.received_parcels)}")
        return 1

if __name__ == "__main__":
    exit(main())
