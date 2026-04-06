from rixmsg.message import Message
from rixmsg.geometry.Pose2D import Pose2D
from rixmsg.standard.Header import Header

class Pose2DStamped(Message):
    def __init__(self):
        self.header = Header()
        self.pose = Pose2D()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.pose)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.pose, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.pose = Message._deserialize_message(buffer, context, Pose2D)

    def hash(self) -> int:
        return [0xe069dc53bf23a3f1, 0x60fb73a105836636]