from rixmsg.message import Message

class TopicInfo(Message):
    def __init__(self):
        self.name = ""
        self.message_hash = [0 for _ in range(2)]

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_array_number(self.message_hash, Message._size_uint64())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_uint64(self.message_hash, buffer, 2, True)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.message_hash = Message._deserialize_uint64(buffer, context, True, 2)

    def hash(self) -> int:
        return [0x8e64137b16fde8ad, 0xe27995f7c4f32229]