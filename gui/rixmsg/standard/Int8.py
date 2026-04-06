from rixmsg.message import Message

class Int8(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_int8()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int8(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int8(buffer, context)

    def hash(self) -> int:
        return [0xf0ec278c8ae6fbfb, 0xaf498e1ee7c0d594]