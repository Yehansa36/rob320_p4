from rixmsg.message import Message
from rixmsg.standard.Header import Header

class Image(Message):
    def __init__(self):
        self.header = Header()
        self.width = 0
        self.height = 0
        self.channels = 0
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_uint32()
        size += Message._size_uint32()
        size += Message._size_uint32()
        size += Message._size_vector_number(self.data, Message._size_uint8())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_uint32(self.width, buffer)
        Message._serialize_uint32(self.height, buffer)
        Message._serialize_uint32(self.channels, buffer)
        Message._serialize_uint8(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.width = Message._deserialize_uint32(buffer, context)
        self.height = Message._deserialize_uint32(buffer, context)
        self.channels = Message._deserialize_uint32(buffer, context)
        self.data = Message._deserialize_uint8(buffer, context, False)

    def hash(self) -> int:
        return [0xeb830a707853bcfb, 0x532a0ab53aac7364]