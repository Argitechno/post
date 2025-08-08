from post_msgs.msg import Parcel

def dict_to_parcel_msg(parcel_id, owner_id, instruction_set_key, data_dict):
    keys = list(data_dict.keys())
    values = [str(v) for v in data_dict.values()]

    msg = Parcel()
    msg.id = parcel_id
    msg.owner_id = owner_id
    msg.instruction_set_key = instruction_set_key
    msg.keys = keys
    msg.values = values
    return msg

def parcel_msg_to_dict(parcel_msg):
    data = dict(zip(parcel_msg.keys, parcel_msg.values))
    return {
        "id": parcel_msg.id,
        "owner_id": parcel_msg.owner_id,
        "instruction_set_key": parcel_msg.instruction_set_key,
        "data": data,
    }

