from post_instruction_sets import register_instruction_set
import copy

@register_instruction_set("dynamic_loop")
def dynamic_loop_instruction_set(parcel, station):
    ttl_str = parcel["data"].get("ttl", None)
    if ttl_str is None:
        station.get_logger().warn(f"Parcel {parcel['id']} missing 'ttl', dropping.")
        return
    
    try:
        ttl = int(ttl_str)
    except ValueError:
        station.get_logger().warn(f"Parcel {parcel['id']} has invalid 'ttl' value '{ttl_str}', dropping.")
        return
    
    if ttl <= 0:
        station.get_logger().info(f"Parcel {parcel['id']} TTL expired, dropping.")
        return

    # Decrement ttl
    ttl -= 1
    parcel["data"]["ttl"] = str(ttl)

    # Determine next destination list and current index
    destinations = parcel["data"].get("destinations", "")
    if not destinations:
        station.get_logger().warn(f"Parcel {parcel['id']} missing 'destinations', dropping.")
        return
    
    dest_list = [d.strip() for d in destinations.split(",") if d.strip()]
    if not dest_list:
        station.get_logger().warn(f"Parcel {parcel['id']} empty 'destinations', dropping.")
        return
    
    # Get current index, default 0
    index_str = parcel["data"].get("destination_index", "0")
    try:
        index = int(index_str)
    except ValueError:
        index = 0
    
    # Pick next destination in loop
    next_dest = dest_list[index % len(dest_list)]
    
    # Update index for next time
    parcel["data"]["destination_index"] = str((index + 1) % len(dest_list))

    station.get_logger().info(f"Parcel {parcel['id']} routing to {next_dest} with TTL {ttl}")

    # Create a copy to avoid side effects (optional)
    parcel_copy = copy.deepcopy(parcel)

    # Send parcel to the topic for that destination
    topic = f"/stations/{next_dest}/incoming_parcels"
    station.send(parcel_copy, destination_topic=topic)
