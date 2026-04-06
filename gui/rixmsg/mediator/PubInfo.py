from rixmsg.message import Message
from rixmsg.mediator.Endpoint import Endpoint
from rixmsg.mediator.TopicInfo import TopicInfo

class PubInfo(Message):
    def __init__(self):
        self.id = 0
        self.node_id = 0
        self.protocol = 0
        self.topic_info = TopicInfo()
        self.endpoint = Endpoint()

    def size(self) -> int:
        size = 0
        size += Message._size_uint64()
        size += Message._size_uint64()
        size += Message._size_uint8()
        size += Message._size_message(self.topic_info)
        size += Message._size_message(self.endpoint)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_uint64(self.node_id, buffer)
        Message._serialize_uint8(self.protocol, buffer)
        Message._serialize_message(self.topic_info, buffer)
        Message._serialize_message(self.endpoint, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.id = Message._deserialize_uint64(buffer, context)
        self.node_id = Message._deserialize_uint64(buffer, context)
        self.protocol = Message._deserialize_uint8(buffer, context)
        self.topic_info = Message._deserialize_message(buffer, context, TopicInfo)
        self.endpoint = Message._deserialize_message(buffer, context, Endpoint)

    def hash(self) -> int:
        return [0x77fbde7d73a67b6c, 0xf920af1abea2993b]