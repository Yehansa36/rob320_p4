from rixmsg.message import Message
from rixmsg.standard.Header import Header

class CompressedImage(Message):
    def __init__(self):
        self.header = Header()
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_number(self.data, Message._size_uint8())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_uint8(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.data = Message._deserialize_uint8(buffer, context, False)

    def hash(self) -> int:
        return [0xcd18494cd9a05543, 0xb8b8bc270304ba83]