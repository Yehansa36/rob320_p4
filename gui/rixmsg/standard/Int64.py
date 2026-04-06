from rixmsg.message import Message

class Int64(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_int64()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int64(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int64(buffer, context)

    def hash(self) -> int:
        return [0x397553115c8f3b8c, 0x07b986684616d69d]