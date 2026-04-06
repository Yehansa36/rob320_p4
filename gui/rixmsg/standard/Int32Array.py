from rixmsg.message import Message

class Int32Array(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_int32())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int32(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_int32(buffer, context, False)

    def hash(self) -> int:
        return [0x9930fa2ea3fcbb0f, 0x99d3e911ee2a6b9f]