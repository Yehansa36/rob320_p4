from rixmsg.message import Message

class Int16(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_int16()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int16(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int16(buffer, context)

    def hash(self) -> int:
        return [0x407381f3637b2776, 0xfbef4993d27d03d5]