from rixmsg.message import Message
from rixmsg.geometry.Twist2D import Twist2D
from rixmsg.standard.Header import Header

class Twist2DStamped(Message):
    def __init__(self):
        self.header = Header()
        self.twist = Twist2D()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.twist)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.twist, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.twist = Message._deserialize_message(buffer, context, Twist2D)

    def hash(self) -> int:
        return [0xb0608b7ec6e4b424, 0xcf703aa6aad285a6]