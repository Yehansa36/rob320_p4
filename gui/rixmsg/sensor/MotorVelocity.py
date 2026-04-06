from rixmsg.message import Message
from rixmsg.standard.Header import Header

class MotorVelocity(Message):
    def __init__(self):
        self.header = Header()
        self.velocity = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_number(self.velocity, Message._size_float())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_float(self.velocity, buffer, len(self.velocity), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.velocity = Message._deserialize_float(buffer, context, False)

    def hash(self) -> int:
        return [0x013ff91a89c54064, 0xd095f9402deba652]