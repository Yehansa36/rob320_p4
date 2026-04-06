from rixmsg.message import Message

class UInt16Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_uint16())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint16(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_uint16(buffer, context, False)

    def hash(self) -> int:
        return [0x0e5e038e3bf6c3d0, 0xf1adf5b00f1ad48e]