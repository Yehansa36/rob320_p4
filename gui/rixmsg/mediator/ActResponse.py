from rixmsg.message import Message
from rixmsg.mediator.ActInfo import ActInfo

class ActResponse(Message):
    def __init__(self):
        self.error = 0
        self.act_info = ActInfo()

    def size(self) -> int:
        size = 0
        size += Message._size_uint8()
        size += Message._size_message(self.act_info)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint8(self.error, buffer)
        Message._serialize_message(self.act_info, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.error = Message._deserialize_uint8(buffer, context)
        self.act_info = Message._deserialize_message(buffer, context, ActInfo)

    def hash(self) -> int:
        return [0xf50b5c0f46968e3a, 0x6b7175cc3a445ad9]