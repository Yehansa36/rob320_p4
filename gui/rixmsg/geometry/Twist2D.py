from rixmsg.message import Message

class Twist2D(Message):
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_float(self.vx, buffer)
        Message._serialize_float(self.vy, buffer)
        Message._serialize_float(self.wz, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.vx = Message._deserialize_float(buffer, context)
        self.vy = Message._deserialize_float(buffer, context)
        self.wz = Message._deserialize_float(buffer, context)

    def hash(self) -> int:
        return [0xd87fbd8e8f8e4037, 0x2cc37c82e4a30bff]