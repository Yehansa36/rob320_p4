from rixmsg.message import Message

class DoubleArray(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_double())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_double(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_double(buffer, context, False)

    def hash(self) -> int:
        return [0x49e8272ebee73432, 0xbc22c3f3845ffc9b]