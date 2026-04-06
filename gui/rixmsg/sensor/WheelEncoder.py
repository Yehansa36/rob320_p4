from rixmsg.message import Message
from rixmsg.standard.Header import Header

class WheelEncoder(Message):
    def __init__(self):
        self.header = Header()
        self.ticks = []
        self.delta_ticks = []
        self.delta_time = 0

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_number(self.ticks, Message._size_int64())
        size += Message._size_vector_number(self.delta_ticks, Message._size_int32())
        size += Message._size_int32()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_int64(self.ticks, buffer, len(self.ticks), False)
        Message._serialize_int32(self.delta_ticks, buffer, len(self.delta_ticks), False)
        Message._serialize_int32(self.delta_time, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.ticks = Message._deserialize_int64(buffer, context, False)
        self.delta_ticks = Message._deserialize_int32(buffer, context, False)
        self.delta_time = Message._deserialize_int32(buffer, context)

    def hash(self) -> int:
        return [0xb1bb3ab289b9324a, 0x8e71f13d72e4a8ae]