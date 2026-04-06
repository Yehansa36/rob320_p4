from rixmsg.message import Message

class UInt32(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint32()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint32(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint32(buffer, context)

    def hash(self) -> int:
        return [0x720d747016f03643, 0x50fda753f6f4e461]