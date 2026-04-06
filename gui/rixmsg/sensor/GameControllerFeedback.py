from rixmsg.message import Message
from rixmsg.standard.Duration import Duration
from rixmsg.standard.Header import Header

class GameControllerFeedback(Message):
    def __init__(self):
        self.header = Header()
        self.intensities = []
        self.durations = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_number(self.intensities, Message._size_float())
        size += Message._size_vector_message(self.durations)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_float(self.intensities, buffer, len(self.intensities), False)
        Message._serialize_message(self.durations, buffer, len(self.durations), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.intensities = Message._deserialize_float(buffer, context, False)
        self.durations = Message._deserialize_message(buffer, context, Duration, False)

    def hash(self) -> int:
        return [0x179e24083020f533, 0x5b33b70a8fc2cd33]