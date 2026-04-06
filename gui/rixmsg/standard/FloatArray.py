from rixmsg.message import Message

class FloatArray(Message):
    def __init__(self):
        self.data = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_number(self.data, Message._size_float())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_float(self.data, buffer, len(self.data), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.data = Message._deserialize_float(buffer, context, False)

    def hash(self) -> int:
        return [0x10e6c98a933406ce, 0x39e752fb9d400379]