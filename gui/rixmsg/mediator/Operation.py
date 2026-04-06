from rixmsg.message import Message

class Operation(Message):
    def __init__(self):
        self.len = 0
        self.opcode = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint32()
        size += Message._size_uint8()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint32(self.len, buffer)
        Message._serialize_uint8(self.opcode, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.len = Message._deserialize_uint32(buffer, context)
        self.opcode = Message._deserialize_uint8(buffer, context)

    def hash(self) -> int:
        return [0x86699b36c0a476cb, 0x3538e0260620e597]