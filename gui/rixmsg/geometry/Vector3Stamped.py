from rixmsg.message import Message
from rixmsg.geometry.Vector3 import Vector3
from rixmsg.standard.Header import Header

class Vector3Stamped(Message):
    def __init__(self):
        self.header = Header()
        self.vector3 = Vector3()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.vector3)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.vector3, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.vector3 = Message._deserialize_message(buffer, context, Vector3)

    def hash(self) -> int:
        return [0x9236c1ec83a06e9f, 0x19da5fd174aed535]