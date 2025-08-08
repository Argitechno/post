from rclpy.node import Node
from queue import Queue

from post_msgs.msg import Parcel
from .parcel_utils import parcel_msg_to_dict, dict_to_parcel_msg

class BaseStation(Node):
    def __init__(self, name):
        super().__init__(name)
        self.parcel_queue = Queue()

        # Subscribe to incoming parcels
        self.subscriber = self.create_subscription(
            Parcel,
            f"stations/{self.get_name()}/incoming_parcels",
            self._parcel_callback,
            10
        )

        self._pub_cache = {}

    def _get_publisher(self, topic):
        if topic not in self._pub_cache :
            self._pub_cache[topic] = self.create_publisher(Parcel, topic, 10)
        return self._pub_cache[topic]

    def _parcel_callback(self, msg):
        parcel = parcel_msg_to_dict(msg)
        self.get_logger().info(f"[{self.get_name()}] Received parcel: {parcel['id']}")
        self.parcel_queue.put(parcel)

    def send(self, parcel, destination_topic=None):
        topic = destination_topic or f"stations/{self.get_name()}/incoming_parcels"
        publisher = self._get_publisher(topic)
        msg = dict_to_parcel_msg(
            parcel["id"], parcel["owner_id"], parcel["instruction_set_key"], parcel["data"]
        )
        self.get_logger().info(f"[{self.get_name()}] Sending parcel {parcel['id']} to {topic}")
        publisher.publish(msg)
