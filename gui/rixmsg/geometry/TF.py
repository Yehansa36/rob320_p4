from rixmsg.message import Message
from rixmsg.geometry.TransformStamped import TransformStamped

class TF(Message):
    def __init__(self):
        self.transforms = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_message(self.transforms)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.transforms, buffer, len(self.transforms), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.transforms = Message._deserialize_message(buffer, context, TransformStamped, False)

    def hash(self) -> int:
        return [0xa260c3d7ccd775ec, 0x176b837161441526]