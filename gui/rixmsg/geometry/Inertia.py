from rixmsg.message import Message
from rixmsg.geometry.Vector3 import Vector3

class Inertia(Message):
    def __init__(self):
        self.mass = 0.0
        self.center_of_mass = Vector3()
        self.ixx = 0.0
        self.ixy = 0.0
        self.ixz = 0.0
        self.iyy = 0.0
        self.iyz = 0.0
        self.izz = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_double()
        size += Message._size_message(self.center_of_mass)
        size += Message._size_double()
        size += Message._size_double()
        size += Message._size_double()
        size += Message._size_double()
        size += Message._size_double()
        size += Message._size_double()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_double(self.mass, buffer)
        Message._serialize_message(self.center_of_mass, buffer)
        Message._serialize_double(self.ixx, buffer)
        Message._serialize_double(self.ixy, buffer)
        Message._serialize_double(self.ixz, buffer)
        Message._serialize_double(self.iyy, buffer)
        Message._serialize_double(self.iyz, buffer)
        Message._serialize_double(self.izz, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.mass = Message._deserialize_double(buffer, context)
        self.center_of_mass = Message._deserialize_message(buffer, context, Vector3)
        self.ixx = Message._deserialize_double(buffer, context)
        self.ixy = Message._deserialize_double(buffer, context)
        self.ixz = Message._deserialize_double(buffer, context)
        self.iyy = Message._deserialize_double(buffer, context)
        self.iyz = Message._deserialize_double(buffer, context)
        self.izz = Message._deserialize_double(buffer, context)

    def hash(self) -> int:
        return [0xab914168d3ceee44, 0x4f58ffe82b66ee05]