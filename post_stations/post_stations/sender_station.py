import sys
import uuid
import random
import rclpy
from rclpy.node import Node
from post_stations.base_station import BaseStation

class SenderStation(BaseStation):
    def __init__(self, name, qos_profile, args):
        super().__init__(name, qos_profile)

        self.destinations = args.destinations.split(",")
        self.mode = args.mode
        self.send_interval_sec = args.send_interval_sec
        self.max_parcels = args.max_parcels

        self.sent_count = 0
        self.round_robin_index = 0
        self.has_broadcasted = False

        self.timer = self.create_timer(self.send_interval_sec, self._send_parcel)

    def _send_parcel(self):
        if self.max_parcels is not None and self.sent_count >= self.max_parcels:
            self.get_logger().info(
                f"[{self.get_name()}] Max parcels {self.max_parcels} sent. Stopping."
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
                        f"[{self.get_name()}] Broadcasted parcel {parcel['id']} to {dest}"
                    )
                self.has_broadcasted = True
                self.sent_count += 1
            else:
                # Already broadcasted, stop timer
                self.get_logger().info(
                    f"[{self.get_name()}] Broadcast once complete. Stopping timer."
                )
                self.timer.cancel()

        elif self.mode == "round_robin":
            dest = self.destinations[self.round_robin_index]
            topic = f"stations/{dest}/incoming_parcels"
            self.send(parcel, destination_topic=topic)
            self.get_logger().info(
                f"[{self.get_name()}] Sent parcel {parcel['id']} to {dest} (round_robin)"
            )
            self.round_robin_index = (self.round_robin_index + 1) % len(self.destinations)
            self.sent_count += 1

        elif self.mode == "random":
            dest = random.choice(self.destinations)
            topic = f"stations/{dest}/incoming_parcels"
            self.send(parcel, destination_topic=topic)
            self.get_logger().info(
                f"[{self.get_name()}] Sent parcel {parcel['id']} to {dest} (random)"
            )
            self.sent_count += 1

        else:
            self.get_logger().error(f"[{self.get_name()}] Unknown mode: {self.mode}")
            self.timer.cancel()

    def _create_parcel(self):
        return {
            "id": str(uuid.uuid4()),
            "owner_id": self.get_name(),
            "instruction_set_key": "default",
            "data": {
                "message": "Hello from SenderStation",
                "timestamp": self.get_clock().now().to_msg().sec,
            },
        }

    def inject_parcel(self, parcel):
        self.send(parcel)
        self.get_logger().info(f"[{self.get_name()}] Injected parcel {parcel['id']}")

def main():
    from post_stations.station_running import main_template
    def add_sender_args(parser):
        parser.add_argument('--mode', default='broadcast_once', choices=['broadcast_once', 'round_robin', 'random'])
        parser.add_argument('--destinations', required=True, help="Comma-separated list of destination station names")
        parser.add_argument('--send_interval_sec', type=float, default=5.0)
        parser.add_argument('--max_parcels', type=int, default=None)
    main_template(SenderStation, default_name="sender_station", add_args_fn=add_sender_args)

if __name__ == "__main__":
    main()
