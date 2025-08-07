import rclpy
from rclpy.executors import SingleThreadedExecutor
import uuid
import time

from post_stations.base_station import BaseStation

def main():
    rclpy.init()
    
    station = BaseStation("loopback")

    executor = SingleThreadedExecutor()
    executor.add_node(station)

    # Send a test parcel to itself after startup
    parcel = {
        "id": str(uuid.uuid4()),
        "owner_id": "test_user",
        "instruction_set_key": "noop",
        "data": {
            "hello": "world",
            "count": 42
        }
    }

    # Wait for subscriptions to be fully ready
    time.sleep(0.5)
    station.send(parcel)  # Send to self

    # Spin for a short time to allow receipt
    print("[TEST] Spinning for 3 seconds to receive parcel...")
    start_time = time.time()
    while time.time() - start_time < 3:
        rclpy.spin_once(station, timeout_sec=0.1)

        if not station.parcel_queue.empty():
            received = station.parcel_queue.get()
            print(f"[TEST] Parcel received in queue: {received}")
            break

    station.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
