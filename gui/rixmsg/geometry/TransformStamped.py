from rixmsg.message import Message
from rixmsg.geometry.Transform import Transform
from rixmsg.standard.Header import Header

class TransformStamped(Message):
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.transform = Transform()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_string(self.child_frame_id)
        size += Message._size_message(self.transform)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_string(self.child_frame_id, buffer)
        Message._serialize_message(self.transform, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.child_frame_id = Message._deserialize_string(buffer, context)
        self.transform = Message._deserialize_message(buffer, context, Transform)

    def hash(self) -> int:
        return [0x5e276c2aba550001, 0xbeb45a3d7e3c5100]