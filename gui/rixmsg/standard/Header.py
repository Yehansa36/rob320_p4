from rixmsg.message import Message
from rixmsg.standard.Time import Time

class Header(Message):
    def __init__(self):
        self.seq = 0
        self.stamp = Time()
        self.frame_id = ""

    def size(self) -> int:
        size = 0
        size += Message._size_uint32()
        size += Message._size_message(self.stamp)
        size += Message._size_string(self.frame_id)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint32(self.seq, buffer)
        Message._serialize_message(self.stamp, buffer)
        Message._serialize_string(self.frame_id, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.seq = Message._deserialize_uint32(buffer, context)
        self.stamp = Message._deserialize_message(buffer, context, Time)
        self.frame_id = Message._deserialize_string(buffer, context)

    def hash(self) -> int:
        return [0xc843ae39514c0d3a, 0x13af07ef9144f643]