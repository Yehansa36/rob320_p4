from rixmsg.message import Message

class Double(Message):
    def __init__(self):
        self.data = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_double()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_double(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_double(buffer, context)

    def hash(self) -> int:
        return [0xb2946cc965c09d4b, 0x2d6ad4c506beeb12]