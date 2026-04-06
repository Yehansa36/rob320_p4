from rixmsg.message import Message
from rixmsg.standard.Header import Header

class GameController(Message):
    def __init__(self):
        self.header = Header()
        self.axes = []
        self.buttons = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_number(self.axes, Message._size_float())
        size += Message._size_vector_number(self.buttons, Message._size_int32())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_float(self.axes, buffer, len(self.axes), False)
        Message._serialize_int32(self.buttons, buffer, len(self.buttons), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.axes = Message._deserialize_float(buffer, context, False)
        self.buttons = Message._deserialize_int32(buffer, context, False)

    def hash(self) -> int:
        return [0x7e1534c7c3f79f57, 0x13a4c5f0b126109e]