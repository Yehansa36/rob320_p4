from rixmsg.message import Message

class Int8Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_int8())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int8(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int8(buffer, context, False)

    def hash(self) -> int:
        return [0x9d063ecb5b6913fc, 0x0dbacca20c51f5cc]