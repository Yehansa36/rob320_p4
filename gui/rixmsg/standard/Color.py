from rixmsg.message import Message

class Color(Message):
    def __init__(self):
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0
        self.a = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_float(self.r, buffer)
        Message._serialize_float(self.g, buffer)
        Message._serialize_float(self.b, buffer)
        Message._serialize_float(self.a, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.r = Message._deserialize_float(buffer, context)
        self.g = Message._deserialize_float(buffer, context)
        self.b = Message._deserialize_float(buffer, context)
        self.a = Message._deserialize_float(buffer, context)

    def hash(self) -> int:
        return [0xf88b69207e8f108c, 0x3824545f5cb73032]