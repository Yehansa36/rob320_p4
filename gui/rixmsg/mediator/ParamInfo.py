from rixmsg.message import Message

class ParamInfo(Message):
    def __init__(self):
        self.name = ""
        self.message_hash = [0 for _ in range(2)]
        self.len = 0

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_array_number(self.message_hash, Message._size_uint64())
        size += Message._size_uint32()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_uint64(self.message_hash, buffer, 2, True)
        Message._serialize_uint32(self.len, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.message_hash = Message._deserialize_uint64(buffer, context, True, 2)
        self.len = Message._deserialize_uint32(buffer, context)

    def hash(self) -> int:
        return [0xe97063b954a0c290, 0x48cd801fd1f332e4]