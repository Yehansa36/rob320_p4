from rixmsg.message import Message
from rixmsg.geometry.Pose import Pose
from rixmsg.standard.Header import Header

class PoseStamped(Message):
    def __init__(self):
        self.header = Header()
        self.pose = Pose()

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
        self.pose = Message._deserialize_message(buffer, context, Pose)

    def hash(self) -> int:
        return [0xab156c3437467a52, 0xb413ce13116d40fc]