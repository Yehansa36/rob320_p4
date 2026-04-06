from rixmsg.message import Message
from rixmsg.geometry.Inertia import Inertia
from rixmsg.standard.Header import Header

class InertiaStamped(Message):
    def __init__(self):
        self.header = Header()
        self.inertia = Inertia()

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_message(self.inertia)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.inertia, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.inertia = Message._deserialize_message(buffer, context, Inertia)

    def hash(self) -> int:
        return [0x253aa1dc0252f540, 0x14e8902090fb63a5]