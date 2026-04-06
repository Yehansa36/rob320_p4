from rixmsg.message import Message
from rixmsg.geometry.Quaternion import Quaternion
from rixmsg.geometry.Vector3 import Vector3
from rixmsg.standard.Header import Header

class IMU(Message):
    def __init__(self):
        self.header = Header()
        self.orientation = Quaternion()
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.orientation)
        size += Message._size_message(self.angular_velocity)
        size += Message._size_message(self.linear_acceleration)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.orientation, buffer)
        Message._serialize_message(self.angular_velocity, buffer)
        Message._serialize_message(self.linear_acceleration, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.orientation = Message._deserialize_message(buffer, context, Quaternion)
        self.angular_velocity = Message._deserialize_message(buffer, context, Vector3)
        self.linear_acceleration = Message._deserialize_message(buffer, context, Vector3)

    def hash(self) -> int:
        return [0xea0a73fa393c53c3, 0x5e980c6ed199f2e5]