from rixmsg.message import Message

class UInt32Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_uint32())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint32(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint32(buffer, context, False)

    def hash(self) -> int:
        return [0xaefe9199dbb27e93, 0xd8ccb06b7c8ad5d4]