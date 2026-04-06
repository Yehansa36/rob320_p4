from rixmsg.message import Message
from rixmsg.geometry.Vector3 import Vector3

class Twist(Message):
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.linear)
        size += Message._size_message(self.angular)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.linear, buffer)
        Message._serialize_message(self.angular, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.linear = Message._deserialize_message(buffer, context, Vector3)
        self.angular = Message._deserialize_message(buffer, context, Vector3)

    def hash(self) -> int:
        return [0x312d32d77af3700a, 0xed8db0e780d13e74]