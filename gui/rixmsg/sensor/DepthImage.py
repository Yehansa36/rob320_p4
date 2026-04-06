from rixmsg.message import Message
from rixmsg.standard.Header import Header

class DepthImage(Message):
    def __init__(self):
        self.header = Header()
        self.width = 0
        self.height = 0
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_uint32()
        size += Message._size_uint32()
        size += Message._size_vector_number(self.data, Message._size_float())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_uint32(self.width, buffer)
        Message._serialize_uint32(self.height, buffer)
        Message._serialize_float(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.width = Message._deserialize_uint32(buffer, context)
        self.height = Message._deserialize_uint32(buffer, context)
        self.data = Message._deserialize_float(buffer, context, False)

    def hash(self) -> int:
        return [0x7b679082dc7b2b53, 0x0db3b8881837cd7e]