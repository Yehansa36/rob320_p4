from rixmsg.message import Message

class Status(Message):
    def __init__(self):
        self.id = 0
        self.error = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint64()
        size += Message._size_uint8()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_uint8(self.error, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.id = Message._deserialize_uint64(buffer, context)
        self.error = Message._deserialize_uint8(buffer, context)

    def hash(self) -> int:
        return [0x7fb8fd2b37424f48, 0xab387927011d828b]