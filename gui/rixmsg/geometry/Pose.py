from rixmsg.message import Message
from rixmsg.geometry.Point import Point
from rixmsg.geometry.Quaternion import Quaternion

class Pose(Message):
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.position)
        size += Message._size_message(self.orientation)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.position, buffer)
        Message._serialize_message(self.orientation, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.position = Message._deserialize_message(buffer, context, Point)
        self.orientation = Message._deserialize_message(buffer, context, Quaternion)

    def hash(self) -> int:
        return [0x5e8de7739d32edb5, 0xb8858a25a14cc5a9]