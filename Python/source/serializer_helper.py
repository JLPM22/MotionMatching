import struct


def read_uint(f):
    return int.from_bytes(f.read(4), "little", signed=False)


def read_float(f):
    return struct.unpack("<f", f.read(4))[0]  # little endian


def read_uint_LEB128(f):
    length = 0
    shift = 0
    while True:
        byte = f.read(1)
        if byte == b"":
            raise EOFError("Unexpected EOF reached")
        byte = byte[0]
        length |= (byte & 0x7F) << shift
        if (byte & 0x80) == 0:
            break
        shift += 7
    return length


def read_string(f):
    # string in BinaryWriter from C# are encoded with a uint LEB128 prefix
    # that indicates the length of the string, then the string itself
    length = read_uint_LEB128(f)
    return struct.unpack("<" + str(length) + "s", f.read(length))[0].decode("utf-8")
