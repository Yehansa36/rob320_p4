from rixmsg.message import Message

class ChannelInt32(Message):
    def __init__(self):
        self.name = ""
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_vector_number(self.data, Message._size_int32())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_int32(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.data = Message._deserialize_int32(buffer, context, False)

    def hash(self) -> int:
        return [0x1fe068ef0a63a5a2, 0x8982aea98e0d5f22]