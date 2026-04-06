from rixmsg.message import Message

class JointState(Message):
    def __init__(self):
        self.name = ""
        self.position = 0.0
        self.velocity = 0.0
        self.effort = 0.0

    def size(self) -> int:
        size = 0
        size += Message._size_string(self.name)
        size += Message._size_double()
        size += Message._size_double()
        size += Message._size_double()
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_string(self.name, buffer)
        Message._serialize_double(self.position, buffer)
        Message._serialize_double(self.velocity, buffer)
        Message._serialize_double(self.effort, buffer)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.name = Message._deserialize_string(buffer, context)
        self.position = Message._deserialize_double(buffer, context)
        self.velocity = Message._deserialize_double(buffer, context)
        self.effort = Message._deserialize_double(buffer, context)

    def hash(self) -> int:
        return [0x431aef87adbd38f1, 0x85684cf3334910d0]