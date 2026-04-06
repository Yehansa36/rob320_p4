from rixmsg.message import Message
from rixmsg.mediator.SubInfo import SubInfo

class PubNotify(Message):
    def __init__(self):
        self.id = 0
        self.connect = False
        self.error = 0
        self.subscribers = []

    def size(self) -> int:
        size = 0
        size += Message._size_uint64()
        size += Message._size_bool()
        size += Message._size_uint8()
        size += Message._size_vector_message(self.subscribers)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_bool(self.connect, buffer)
        Message._serialize_uint8(self.error, buffer)
        Message._serialize_message(self.subscribers, buffer, len(self.subscribers), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.id = Message._deserialize_uint64(buffer, context)
        self.connect = Message._deserialize_bool(buffer, context)
        self.error = Message._deserialize_uint8(buffer, context)
        self.subscribers = Message._deserialize_message(buffer, context, SubInfo, False)

    def hash(self) -> int:
        return [0xecd2eab46fd6c2ab, 0x8c29ce8379c80c39]