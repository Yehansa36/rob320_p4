from rixmsg.message import Message
from rixmsg.geometry.Quaternion import Quaternion
from rixmsg.geometry.Vector3 import Vector3

class Transform(Message):
    def __init__(self):
        self.translation = Vector3()
        self.rotation = Quaternion()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.translation)
        size += Message._size_message(self.rotation)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.translation, buffer)
        Message._serialize_message(self.rotation, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.translation = Message._deserialize_message(buffer, context, Vector3)
        self.rotation = Message._deserialize_message(buffer, context, Quaternion)

    def hash(self) -> int:
        return [0x7c6b915aa032ce4a, 0xcb20e26fd99a8832]