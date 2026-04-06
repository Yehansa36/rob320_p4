from rixmsg.message import Message
from rixmsg.mediator.Endpoint import Endpoint

class NodeInfo(Message):
    def __init__(self):
        self.name = ""
        self.id = 0
        self.machine_id = 0
        self.protocol = 0
        self.endpoint = Endpoint()

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_uint64()
        size += Message._size_uint64()
        size += Message._size_uint8()
        size += Message._size_message(self.endpoint)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_uint64(self.machine_id, buffer)
        Message._serialize_uint8(self.protocol, buffer)
        Message._serialize_message(self.endpoint, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.id = Message._deserialize_uint64(buffer, context)
        self.machine_id = Message._deserialize_uint64(buffer, context)
        self.protocol = Message._deserialize_uint8(buffer, context)
        self.endpoint = Message._deserialize_message(buffer, context, Endpoint)

    def hash(self) -> int:
        return [0x4fade8c3f2179b5e, 0x9d84b6c6e3719a8a]