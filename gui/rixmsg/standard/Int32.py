from rixmsg.message import Message

class Int32(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_int32()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int32(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int32(buffer, context)

    def hash(self) -> int:
        return [0x36ca57c416807f18, 0xae2f91cbf1adfbc9]