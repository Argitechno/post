from post_stations.base_station import BaseStation
from post_instruction_sets import instruction_sets
from post_stations.parcel_utils import parcel_msg_to_dict

class ProcessingStation(BaseStation):
    def __init__(self, name):
        super().__init__(name)

    def _parcel_callback(self, msg):
        parcel = parcel_msg_to_dict(msg)
        self.get_logger().info(f"[{self.get_name()}] Processing parcel {parcel['id']} with instruction set {parcel['instruction_set_key']}")

        instr_key = parcel.get("instruction_set_key", "relay")
        instr = instruction_sets.get(instr_key)

        if not instr:
            self.get_logger().warn(f"[{self.get_name()}] Unknown instruction set key: {instr_key}, dropping parcel {parcel['id']}")
            return

        instr(parcel, self)

def main():
    import rclpy
    import asyncio

    rclpy.init()
    node = ProcessingStation("processing_station")
    async def runner():
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=float(0))
                await asyncio.sleep(0.001)  # Small delay for async tasks
        except SystemExit:
            rclpy.logging.get_logger("Quitting").info('Done')
        finally:
            node.destroy_node()
            rclpy.shutdown()

    asyncio.run(runner())

if __name__ == "__main__":
    main()
