from rixmsg.message import Message
from rixmsg.geometry.Point import Point
from rixmsg.sensor.ChannelFloat import ChannelFloat
from rixmsg.standard.Header import Header

class PointCloud(Message):
    def __init__(self):
        self.header = Header()
        self.points = []
        self.channels = []

    def size(self) -> int:
        size = 0
        size += Message._size_message(self.header)
        size += Message._size_vector_message(self.points)
        size += Message._size_vector_message(self.channels)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.header, buffer)
        Message._serialize_message(self.points, buffer, len(self.points), False)
        Message._serialize_message(self.channels, buffer, len(self.channels), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.header = Message._deserialize_message(buffer, context, Header)
        self.points = Message._deserialize_message(buffer, context, Point, False)
        self.channels = Message._deserialize_message(buffer, context, ChannelFloat, False)

    def hash(self) -> int:
        return [0x8bca33deedbad17d, 0xc1e284d18537d76a]