from rixmsg.message import Message

class Endpoint(Message):
    def __init__(self):
        self.port = 0
        self.address = ""

    def size(self) -> int:
        size = 0
        size += Message._size_uint16()
        size += Message._size_string(self.address)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint16(self.port, buffer)
        Message._serialize_string(self.address, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.port = Message._deserialize_uint16(buffer, context)
        self.address = Message._deserialize_string(buffer, context)

    def hash(self) -> int:
        return [0x90624384b177200d, 0xe9ef37ae73ed079f]