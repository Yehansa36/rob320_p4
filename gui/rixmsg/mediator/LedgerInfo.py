from rixmsg.message import Message
from rixmsg.mediator.ActInfo import ActInfo
from rixmsg.mediator.NodeInfo import NodeInfo
from rixmsg.mediator.PubInfo import PubInfo
from rixmsg.mediator.SrvInfo import SrvInfo
from rixmsg.mediator.SubInfo import SubInfo

class LedgerInfo(Message):
    def __init__(self):
        self.nodes = []
        self.publishers = []
        self.subscribers = []
        self.services = []
        self.actions = []

    def size(self) -> int:
        size = 0
        size += Message._size_vector_message(self.nodes)
        size += Message._size_vector_message(self.publishers)
        size += Message._size_vector_message(self.subscribers)
        size += Message._size_vector_message(self.services)
        size += Message._size_vector_message(self.actions)
        return size
    
    def serialize(self, buffer: bytearray) -> None:
        Message._serialize_message(self.nodes, buffer, len(self.nodes), False)
        Message._serialize_message(self.publishers, buffer, len(self.publishers), False)
        Message._serialize_message(self.subscribers, buffer, len(self.subscribers), False)
        Message._serialize_message(self.services, buffer, len(self.services), False)
        Message._serialize_message(self.actions, buffer, len(self.actions), False)

    def deserialize(self, buffer: bytearray, context: dict) -> None:
        self.nodes = Message._deserialize_message(buffer, context, NodeInfo, False)
        self.publishers = Message._deserialize_message(buffer, context, PubInfo, False)
        self.subscribers = Message._deserialize_message(buffer, context, SubInfo, False)
        self.services = Message._deserialize_message(buffer, context, SrvInfo, False)
        self.actions = Message._deserialize_message(buffer, context, ActInfo, False)

    def hash(self) -> int:
        return [0x7f118d3df130fda2, 0xb12f54fa15d2cae5]