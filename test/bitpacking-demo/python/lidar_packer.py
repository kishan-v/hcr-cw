import struct
import json

def pack_lidar(data : str) -> bytes:
    """Bitpack the lidar data"""

    packed = bytearray()  # buffer
    payload = json.loads(data)  # ROS2 message
    if "box_vals" not in payload: 
        raise ValueError("Packer received data without Box Vals in it!")

    bools = payload['box_vals']  # box vals extracted

    # Increment by 8 each time (for bitpacking into bytes)
    for i in range(0, len(bools), 8):
        byte = 0
        chunk = bools[i:i+8]
        for j, b, in enumerate(chunk):
            # OR and shift
            byte |= (1 if b else 0) << j
        # Add the byte to the bytearray. Make sure it's clear this is
        # big-endian!
        packed.append(byte)

    dims = payload['world_dims']
    header_data = struct.pack('iiifL', dims['width'], dims['depth'], dims['height'], dims['step_size'], payload['timestamp'])

    # Concatenate the struct-packed worlddims and timestamp to the bitpacked box_vals
    full_data = header_data + packed
    return full_data
