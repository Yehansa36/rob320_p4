from rixmsg.message import Message

class String(Message):
    def __init__(self):
        self.data = ""

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.data)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_string(buffer, context)

    def hash(self) -> int:
        return [0x607fa99bddbe73e0, 0x32708827f11f4800]