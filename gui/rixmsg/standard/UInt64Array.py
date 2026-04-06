from rixmsg.message import Message

class UInt64Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_uint64())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint64(buffer, context, False)

    def hash(self) -> int:
        return [0x43fdd7613bd8a3ee, 0x1b2702395df92a31]