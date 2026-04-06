from rixmsg.message import Message

class UInt8Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_uint8())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint8(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint8(buffer, context, False)

    def hash(self) -> int:
        return [0x982050f13f28b0b5, 0xb92dba42d84dbc13]