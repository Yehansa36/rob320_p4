from rixmsg.message import Message

class ChannelFloat(Message):
    def __init__(self):
        self.name = ""
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_vector_number(self.data, Message._size_float())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_float(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.data = Message._deserialize_float(buffer, context, False)

    def hash(self) -> int:
        return [0xc97782d7135864fe, 0x5d5322310654455c]