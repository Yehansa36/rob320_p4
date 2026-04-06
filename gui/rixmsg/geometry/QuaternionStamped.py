from rixmsg.message import Message
from rixmsg.geometry.Quaternion import Quaternion
from rixmsg.standard.Header import Header

class QuaternionStamped(Message):
    def __init__(self):
        self.header = Header()
        self.quaternion = Quaternion()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.quaternion)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.quaternion, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.quaternion = Message._deserialize_message(buffer, context, Quaternion)

    def hash(self) -> int:
        return [0x54fe0096d6e750e6, 0xcbd823cbff521c71]