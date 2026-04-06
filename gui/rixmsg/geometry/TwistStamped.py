from rixmsg.message import Message
from rixmsg.geometry.Twist import Twist
from rixmsg.standard.Header import Header

class TwistStamped(Message):
    def __init__(self):
        self.header = Header()
        self.twist = Twist()

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
        self.twist = Message._deserialize_message(buffer, context, Twist)

    def hash(self) -> int:
        return [0xbdb231a752ca29d0, 0x4e60948f323a05be]