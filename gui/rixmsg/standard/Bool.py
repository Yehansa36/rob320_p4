from rixmsg.message import Message

class Bool(Message):
    def __init__(self):
        self.data = False

    def size(self) -> int:
        size = 0
        size += Message._size_bool()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_bool(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_bool(buffer, context)

    def hash(self) -> int:
        return [0x64f2269e8c9b99d6, 0x46cb08359ac45ab2]