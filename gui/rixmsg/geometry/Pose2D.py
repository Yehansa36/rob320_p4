from rixmsg.message import Message

class Pose2D(Message):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_float(self.x, buffer)
        Message._serialize_float(self.y, buffer)
        Message._serialize_float(self.theta, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.x = Message._deserialize_float(buffer, context)
        self.y = Message._deserialize_float(buffer, context)
        self.theta = Message._deserialize_float(buffer, context)

    def hash(self) -> int:
        return [0xe6921cbc9b9f8376, 0x5ab0f8a78d45b88f]