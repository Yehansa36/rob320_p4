from rixmsg.message import Message
from rixmsg.geometry.Point import Point
from rixmsg.standard.Header import Header

class PointStamped(Message):
    def __init__(self):
        self.header = Header()
        self.point = Point()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.point)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.point, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.point = Message._deserialize_message(buffer, context, Point)

    def hash(self) -> int:
        return [0x28b887e9284a56c3, 0xd334f1acfc391d26]