from rixmsg.message import Message

class Duration(Message):
    def __init__(self):
        self.sec = 0
        self.nsec = 0

    def size(self) -> int:
        size = 0
        size += Message._size_int32()
        size += Message._size_int32()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_int32(self.sec, buffer)
        Message._serialize_int32(self.nsec, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.sec = Message._deserialize_int32(buffer, context)
        self.nsec = Message._deserialize_int32(buffer, context)

    def hash(self) -> int:
        return [0x3cfabdd6930400b6, 0x2301ecce2a9d00f6]