import rclpy
from rclpy.executors import MultiThreadedExecutor
import uuid
import time

from post_stations.base_station import BaseStation

def main():
    rclpy.init()

    sender = BaseStation("sender")
    receiver = BaseStation("receiver")

    executor = MultiThreadedExecutor()
    executor.add_node(sender)
    executor.add_node(receiver)

    # Compose a parcel from sender to receiver
    parcel = {
        "id": str(uuid.uuid4()),
        "owner_id": "user_sender",
        "instruction_set_key": "forward",
        "data": {
            "msg": "Hello from sender",
            "value": 123,
        }
    }

    # Wait briefly for subscriptions to settle
    time.sleep(0.5)

    # Send parcel to receiver's topic explicitly
    receiver_topic = f"stations/receiver/incoming_parcels"
    sender.send(parcel, destination_topic=receiver_topic)

    print("[TEST] Spinning for 3 seconds to allow receiver to get parcel...")
    start_time = time.time()
    received = None
    while time.time() - start_time < 3:
        rclpy.spin_once(sender, timeout_sec=0.1)
        rclpy.spin_once(receiver, timeout_sec=0.1)

        if not receiver.parcel_queue.empty():
            received = receiver.parcel_queue.get()
            print(f"[TEST] Receiver got parcel: {received}")
            break

    sender.destroy_node()
    receiver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
