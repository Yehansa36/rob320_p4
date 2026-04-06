from rixmsg.message import Message

class Int64Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_int64())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int64(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int64(buffer, context, False)

    def hash(self) -> int:
        return [0xe404e0ae60673fb9, 0x8febd3535775de6c]