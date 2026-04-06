from rixmsg.message import Message
from rixmsg.mediator.SrvInfo import SrvInfo

class SrvResponse(Message):
    def __init__(self):
        self.error = 0
        self.srv_info = SrvInfo()

    def size(self) -> int:
        size = 0
        size += Message._size_uint8()
        size += Message._size_message(self.srv_info)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint8(self.error, buffer)
        Message._serialize_message(self.srv_info, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.error = Message._deserialize_uint8(buffer, context)
        self.srv_info = Message._deserialize_message(buffer, context, SrvInfo)

    def hash(self) -> int:
        return [0xea5c0f5bb06c7b42, 0xad127f3568ed78ef]