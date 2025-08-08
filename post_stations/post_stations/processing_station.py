from post_stations import BaseStation
from post_instruction_sets import instruction_sets
from post_stations import parcel_msg_to_dict

class ProcessingStation(BaseStation):
    def __init__(self, name,  qos_profile, args):
        super().__init__(name, qos_profile)

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
    from post_stations import main_template
    main_template(ProcessingStation, "processing_station")

if __name__ == "__main__":
    main()
