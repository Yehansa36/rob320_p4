from rixmsg.message import Message
from rixmsg.mediator.PubInfo import PubInfo

class SubNotify(Message):
    def __init__(self):
        self.id = 0
        self.connect = False
        self.error = 0
        self.publishers = []

    def size(self) -> int:
        size = 0
        size += Message._size_uint64()
        size += Message._size_bool()
        size += Message._size_uint8()
        size += Message._size_vector_message(self.publishers)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_uint64(self.id, buffer)
        Message._serialize_bool(self.connect, buffer)
        Message._serialize_uint8(self.error, buffer)
        Message._serialize_message(self.publishers, buffer, len(self.publishers), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.id = Message._deserialize_uint64(buffer, context)
        self.connect = Message._deserialize_bool(buffer, context)
        self.error = Message._deserialize_uint8(buffer, context)
        self.publishers = Message._deserialize_message(buffer, context, PubInfo, False)

    def hash(self) -> int:
        return [0xbd39de770bc7b08d, 0x40ae21673af29472]