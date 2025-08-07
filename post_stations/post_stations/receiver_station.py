import sys
import rclpy
from post_stations.base_station import BaseStation

class ReceiverStation(BaseStation):
    def __init__(self, name):
        super().__init__(name)

    def _parcel_callback(self, msg):
        parcel = self.parcel_msg_to_dict(msg)
        self.get_logger().info(
            f"[{self.name}] Received parcel: {parcel['id']} from {parcel['owner_id']}"
        )
        # Put parcel into internal queue for possible later processing
        self.parcel_queue.put(parcel)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run post_stations receiver_station <name>")
        return

    name = sys.argv[1]
    station = ReceiverStation(name)

    try:
        rclpy.spin(station)
    except KeyboardInterrupt:
        pass

    station.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
