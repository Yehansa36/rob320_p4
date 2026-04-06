from rixmsg.message import Message
from rixmsg.sensor.JointState import JointState
from rixmsg.standard.Time import Time

class JS(Message):
    def __init__(self):
        self.stamp = Time()
        self.joint_states = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.stamp)
        size += Message._size_vector_message(self.joint_states)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.stamp, buffer)
        Message._serialize_message(self.joint_states, buffer, len(self.joint_states), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.stamp = Message._deserialize_message(buffer, context, Time)
        self.joint_states = Message._deserialize_message(buffer, context, JointState, False)

    def hash(self) -> int:
        return [0x77f3eba99ee8ea7a, 0x3696f4f8df4b9ee0]