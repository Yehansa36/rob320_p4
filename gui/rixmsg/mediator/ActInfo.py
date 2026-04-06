from rixmsg.message import Message
from rixmsg.mediator.Endpoint import Endpoint

class ActInfo(Message):
    def __init__(self):
        self.name = ""
        self.id = 0
        self.node_id = 0
        self.protocol = 0
        self.endpoint = Endpoint()
        self.goal_hash = [0 for _ in range(2)]
        self.feedback_hash = [0 for _ in range(2)]
        self.result_hash = [0 for _ in range(2)]

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_uint64()
        size += Message._size_uint64()
        size += Message._size_uint8()
        size += Message._size_message(self.endpoint)
        size += Message._size_array_number(self.goal_hash, Message._size_uint64())
        size += Message._size_array_number(self.feedback_hash, Message._size_uint64())
        size += Message._size_array_number(self.result_hash, Message._size_uint64())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_uint64(self.node_id, buffer)
        Message._serialize_uint8(self.protocol, buffer)
        Message._serialize_message(self.endpoint, buffer)
        Message._serialize_uint64(self.goal_hash, buffer, 2, True)
        Message._serialize_uint64(self.feedback_hash, buffer, 2, True)
        Message._serialize_uint64(self.result_hash, buffer, 2, True)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.id = Message._deserialize_uint64(buffer, context)
        self.node_id = Message._deserialize_uint64(buffer, context)
        self.protocol = Message._deserialize_uint8(buffer, context)
        self.endpoint = Message._deserialize_message(buffer, context, Endpoint)
        self.goal_hash = Message._deserialize_uint64(buffer, context, True, 2)
        self.feedback_hash = Message._deserialize_uint64(buffer, context, True, 2)
        self.result_hash = Message._deserialize_uint64(buffer, context, True, 2)

    def hash(self) -> int:
        return [0x32185eb2b22b1501, 0x23e994315b97981a]