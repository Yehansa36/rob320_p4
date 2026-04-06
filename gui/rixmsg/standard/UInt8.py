from rixmsg.message import Message

class UInt8(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint8()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint8(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint8(buffer, context)

    def hash(self) -> int:
        return [0x305aa7e44b77bf4a, 0x08c011c28ca78f60]