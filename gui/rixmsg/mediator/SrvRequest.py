from rixmsg.message import Message

class SrvRequest(Message):
    def __init__(self):
        self.node_id = 0
        self.id = 0
        self.name = ""
        self.protocol = 0
        self.request_hash = [0 for _ in range(2)]
        self.response_hash = [0 for _ in range(2)]

    def size(self) -> int:
        size = 0
        size += Message._size_uint64()
        size += Message._size_uint64()
        size += Message._size_string(self.name)
        size += Message._size_uint8()
        size += Message._size_array_number(self.request_hash, Message._size_uint64())
        size += Message._size_array_number(self.response_hash, Message._size_uint64())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.node_id, buffer)
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_string(self.name, buffer)
        Message._serialize_uint8(self.protocol, buffer)
        Message._serialize_uint64(self.request_hash, buffer, 2, True)
        Message._serialize_uint64(self.response_hash, buffer, 2, True)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.node_id = Message._deserialize_uint64(buffer, context)
        self.id = Message._deserialize_uint64(buffer, context)
        self.name = Message._deserialize_string(buffer, context)
        self.protocol = Message._deserialize_uint8(buffer, context)
        self.request_hash = Message._deserialize_uint64(buffer, context, True, 2)
        self.response_hash = Message._deserialize_uint64(buffer, context, True, 2)

    def hash(self) -> int:
        return [0x58bc40d436413214, 0x28322f4a4e62bf17]