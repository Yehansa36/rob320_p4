from rixmsg.message import Message

class Float(Message):
    def __init__(self):
        self.data = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_float()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_float(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_float(buffer, context)

    def hash(self) -> int:
        return [0x0c7cd92d8368fc82, 0x903c9572dca1da64]