from rixmsg.message import Message

class Vector3(Message):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_float(self.x, buffer)
        Message._serialize_float(self.y, buffer)
        Message._serialize_float(self.z, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.x = Message._deserialize_float(buffer, context)
        self.y = Message._deserialize_float(buffer, context)
        self.z = Message._deserialize_float(buffer, context)

    def hash(self) -> int:
        return [0x04ebe2ae6d8ee118, 0x670a3ed6f5a9dc4a]