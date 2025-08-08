from post_stations.base_station import BaseStation
from post_stations.parcel_utils import parcel_msg_to_dict

class ReceiverStation(BaseStation):
    def __init__(self, name,  qos_profile, args):
        super().__init__(name, qos_profile)

    def _parcel_callback(self, msg):
        parcel = parcel_msg_to_dict(msg)
        self.get_logger().info(
            f"[{self.get_name()}] Received parcel: {parcel['id']} from {parcel['owner_id']}"
        )
        # Put parcel into internal queue for possible later processing
        self.parcel_queue.put(parcel)


def main():
    from post_stations.station_running import main_template
    main_template(ReceiverStation, "receiver_station")

if __name__ == "__main__":
    main()
