from . import register_instruction_set

@register_instruction_set("relay")
def relay_instruction_set(parcel, station):
    # Simply forward parcel to default destination
    station.get_logger().info(f"[relay] Forwarding parcel {parcel['id']}")
    station.send(parcel)

@register_instruction_set("filter_hello")
def filter_hello_instruction_set(parcel, station):
    # Drop parcels unless they have data['message'] == 'hello'
    msg = parcel["data"].get("message", "")
    if msg.lower() == "hello":
        station.get_logger().info(f"[filter_hello] Forwarding parcel {parcel['id']}")
        station.send(parcel)
    else:
        station.get_logger().info(f"[filter_hello] Dropping parcel {parcel['id']} with message: {msg}")
