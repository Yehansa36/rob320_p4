from rixmsg.message import Message

class UInt16(Message):
    def __init__(self):
        self.data = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint16()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint16(self.data, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint16(buffer, context)

    def hash(self) -> int:
        return [0x8594d6530e9fe1a2, 0x42298ca259112662]