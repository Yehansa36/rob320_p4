from rixmsg.message import Message
from rixmsg.standard.Header import Header

class LaserScan(Message):
    def __init__(self):
        self.header = Header()
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []
        self.intensities = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_float()
        size += Message._size_vector_number(self.ranges, Message._size_float())
        size += Message._size_vector_number(self.intensities, Message._size_float())
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_float(self.angle_min, buffer)
        Message._serialize_float(self.angle_max, buffer)
        Message._serialize_float(self.angle_increment, buffer)
        Message._serialize_float(self.time_increment, buffer)
        Message._serialize_float(self.scan_time, buffer)
        Message._serialize_float(self.range_min, buffer)
        Message._serialize_float(self.range_max, buffer)
        Message._serialize_float(self.ranges, buffer, len(self.ranges), False)
        Message._serialize_float(self.intensities, buffer, len(self.intensities), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.angle_min = Message._deserialize_float(buffer, context)
        self.angle_max = Message._deserialize_float(buffer, context)
        self.angle_increment = Message._deserialize_float(buffer, context)
        self.time_increment = Message._deserialize_float(buffer, context)
        self.scan_time = Message._deserialize_float(buffer, context)
        self.range_min = Message._deserialize_float(buffer, context)
        self.range_max = Message._deserialize_float(buffer, context)
        self.ranges = Message._deserialize_float(buffer, context, False)
        self.intensities = Message._deserialize_float(buffer, context, False)

    def hash(self) -> int:
        return [0xc035b0c601092d80, 0x3a3fb222f8dbb35b]