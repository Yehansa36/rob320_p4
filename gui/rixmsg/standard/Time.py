from rixmsg.message import Message

class Time(Message):
    def __init__(self):
        self.sec = 0
        self.nsec = 0

    def size(self) -> int:
        size = 0
        size += Message._size_uint32()
        size += Message._size_uint32()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint32(self.sec, buffer)
        Message._serialize_uint32(self.nsec, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.sec = Message._deserialize_uint32(buffer, context)
        self.nsec = Message._deserialize_uint32(buffer, context)

    def hash(self) -> int:
        return [0x75f3deb6300bb8bd, 0x6f4c9899240ce7f4]