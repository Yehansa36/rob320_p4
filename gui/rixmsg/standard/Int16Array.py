from rixmsg.message import Message

class Int16Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_int16())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int16(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int16(buffer, context, False)

    def hash(self) -> int:
        return [0x2e757c9537a11e31, 0xa32e271ce88c9848]