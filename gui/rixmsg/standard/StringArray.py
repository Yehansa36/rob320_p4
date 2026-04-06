from rixmsg.message import Message

class StringArray(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_string(self.data)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_string(buffer, context, False)

    def hash(self) -> int:
        return [0xf4562dc36be2b181, 0x5db9d16dfcb0f9d1]