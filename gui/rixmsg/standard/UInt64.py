from rixmsg.message import Message

class UInt64(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint64()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint64(buffer, context)

    def hash(self) -> int:
        return [0x1b7aa3d7870add14, 0xf4a635c2538f2246]