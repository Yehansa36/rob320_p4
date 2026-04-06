from rixmsg.message_base import MessageBase
from rixmsg.mediator.Endpoint import Endpoint

class NodeRegister(MessageBase):
    def __init__(self):
        self.machine_id = 0
        self.id = 0
        self.name = ""
        self.endpoint = Endpoint()

    def size(self) -> int:
        size = 0
        size += MessageBase._size_uint64()
        size += MessageBase._size_uint64()
        size += MessageBase._size_string(self.name)
        size += MessageBase._size_custom(self.endpoint)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        MessageBase._serialize_uint64(self.machine_id, buffer)
        MessageBase._serialize_uint64(self.id, buffer)
        MessageBase._serialize_string(self.name, buffer)
        MessageBase._serialize_custom(self.endpoint, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.machine_id = MessageBase._deserialize_uint64(buffer, context)
        self.id = MessageBase._deserialize_uint64(buffer, context)
        self.name = MessageBase._deserialize_string(buffer, context)
        self.endpoint = MessageBase._deserialize_custom(buffer, context, Endpoint)

    def hash(self) -> int:
        return [0xf4242241b6d39059, 0xf32f049f07439335]