import sys
import uuid
import random
import rclpy
from rclpy.node import Node
from post_stations.base_station import BaseStation

class SenderStation(BaseStation):
    def __init__(
        self,
        name,
        destinations,
        mode="broadcast_once",
        send_interval_sec=5.0,
        max_parcels=None,
    ):
        super().__init__(name)

        if not destinations:
            raise ValueError("At least one destination must be specified")

        self.destinations = destinations
        self.mode = mode
        self.send_interval_sec = send_interval_sec
        self.max_parcels = max_parcels  # None means unlimited

        self.sent_count = 0
        self.round_robin_index = 0
        self.has_broadcasted = False

        self.timer = self.create_timer(self.send_interval_sec, self._send_parcel)

    def _send_parcel(self):
        if self.max_parcels is not None and self.sent_count >= self.max_parcels:
            self.get_logger().info(
                f"[{self.name}] Max parcels {self.max_parcels} sent. Stopping."
            )
            self.timer.cancel()
            return

        parcel = self._create_parcel()

        if self.mode == "broadcast_once":
            if not self.has_broadcasted:
                for dest in self.destinations:
                    topic = f"stations/{dest}/incoming_parcels"
                    self.send(parcel, destination_topic=topic)
                    self.get_logger().info(
                        f"[{self.name}] Broadcasted parcel {parcel['id']} to {dest}"
                    )
                self.has_broadcasted = True
                self.sent_count += 1
            else:
                # Already broadcasted, stop timer
                self.get_logger().info(
                    f"[{self.name}] Broadcast once complete. Stopping timer."
                )
                self.timer.cancel()

        elif self.mode == "round_robin":
            dest = self.destinations[self.round_robin_index]
            topic = f"stations/{dest}/incoming_parcels"
            self.send(parcel, destination_topic=topic)
            self.get_logger().info(
                f"[{self.name}] Sent parcel {parcel['id']} to {dest} (round_robin)"
            )
            self.round_robin_index = (self.round_robin_index + 1) % len(self.destinations)
            self.sent_count += 1

        elif self.mode == "random":
            dest = random.choice(self.destinations)
            topic = f"stations/{dest}/incoming_parcels"
            self.send(parcel, destination_topic=topic)
            self.get_logger().info(
                f"[{self.name}] Sent parcel {parcel['id']} to {dest} (random)"
            )
            self.sent_count += 1

        else:
            self.get_logger().error(f"[{self.name}] Unknown mode: {self.mode}")
            self.timer.cancel()

    def _create_parcel(self):
        return {
            "id": str(uuid.uuid4()),
            "owner_id": self.name,
            "instruction_set_key": "default",
            "data": {
                "message": "Hello from SenderStation",
                "timestamp": self.get_clock().now().to_msg().sec,
            },
        }

    def inject_parcel(self, parcel):
        self.send(parcel)
        self.get_logger().info(f"[{self.name}] Injected parcel {parcel['id']}")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 4:
        print(
            "Usage: ros2 run post_stations sender_station <name> <mode> <destinations_comma_separated> [interval_sec] [max_parcels]"
        )
        print("Modes: broadcast_once, round_robin, random")
        return

    name = sys.argv[1]
    mode = sys.argv[2]
    destinations = sys.argv[3].split(",")
    send_interval = float(sys.argv[4]) if len(sys.argv) >= 5 else 5.0
    max_parcels = int(sys.argv[5]) if len(sys.argv) >= 6 else None

    station = SenderStation(
        name,
        destinations,
        mode=mode,
        send_interval_sec=send_interval,
        max_parcels=max_parcels,
    )

    try:
        rclpy.spin(station)
    except KeyboardInterrupt:
        pass

    station.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
