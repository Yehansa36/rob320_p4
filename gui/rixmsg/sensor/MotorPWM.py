from rixmsg.message import Message
from rixmsg.standard.Header import Header

class MotorPWM(Message):
    def __init__(self):
        self.header = Header()
        self.pwm = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_number(self.pwm, Message._size_float())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_float(self.pwm, buffer, len(self.pwm), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.pwm = Message._deserialize_float(buffer, context, False)

    def hash(self) -> int:
        return [0x357281f6c9e66107, 0x79743927d74d0e85]