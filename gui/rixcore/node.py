import random
import os
import signal
import struct
import socket
from typing import Tuple

from rixcore.common import (
    RIXHUB_PORT,
    send_message_with_opcode,
    send_message_with_opcode_no_response,
    OPCODE,
)
from rixmsg.mediator.PubInfo import PubInfo
from rixmsg.mediator.SubInfo import SubInfo
from rixcore.publisher import Publisher
from rixcore.subscriber import Subscriber
from rixmsg.mediator.NodeInfo import NodeInfo


class Node:
    _impl = None  # static NodeImpl
    _initialized = False

    @staticmethod
    def create(
        name: str, rixhub_endpoint: Tuple[str, int] = ("127.0.0.1", RIXHUB_PORT)
    ) -> "Node":
        info = NodeInfo()
        info.id = Node._generateID()
        info.machine_id = Node._getMachineID()
        info.name = name

        return Node(info, rixhub_endpoint)

    def __init__(
        self,
        info: NodeInfo,
        rixhub_endpoint: Tuple[str, int] = ("127.0.0.1", RIXHUB_PORT),
    ):
        self.info = info
        self.rixhub_endpoint = rixhub_endpoint
        self.shutdown_flag = False
        self.components = set()

        client = socket.create_connection(self.rixhub_endpoint)
        if not send_message_with_opcode(
            client, self.info, OPCODE.NODE_REGISTER, self.rixhub_endpoint
        ):
            self.shutdown()

    def __del__(self):
        self.shutdown()
        client = socket.create_connection(self.rixhub_endpoint)
        send_message_with_opcode_no_response(
            client, self.info, OPCODE.NODE_DEREGISTER, self.rixhub_endpoint
        )

    def spin(self) -> None:
        try:
            while self.ok():
                remove_list = []
                for component in self.components:
                    if not component.ok():
                        remove_list.append(component)
                        continue
                    component._spin_once()
                for component in remove_list:
                    self.components.remove(component)
        except KeyboardInterrupt as e:
            self.shutdown()

    def shutdown(
        self,
    ) -> None:
        self.shutdown_flag = True

    def ok(
        self,
    ) -> bool:
        return not self.shutdown_flag

    def create_publisher(
        self, TMsg: any, topic: str, endpoint: Tuple[str, int] = ("127.0.0.1", 0)
    ) -> Publisher:
        info = PubInfo()
        info.id = Node._generateID()
        info.node_id = self.info.id
        info.topic_info.name = topic
        info.topic_info.message_hash = TMsg().hash()

        server = socket.create_server(endpoint)
        server_endpoint = server.getsockname()
        info.endpoint.address = server_endpoint[0]
        info.endpoint.port = server_endpoint[1]

        pub = Publisher(info, server, self.rixhub_endpoint)
        self.components.add(pub)
        return pub

    def create_subscriber(
        self,
        TMsg: any,
        topic: str,
        cb: callable,
        endpoint: Tuple[str, int] = ("127.0.0.1", 0),
    ) -> Subscriber:
        info = SubInfo()
        info.id = Node._generateID()
        info.node_id = self.info.id
        info.topic_info.name = topic
        info.topic_info.message_hash = TMsg().hash()

        server = socket.create_server(endpoint)
        server_endpoint = server.getsockname()
        info.endpoint.address = server_endpoint[0]
        info.endpoint.port = server_endpoint[1]

        sub = Subscriber(info, server, self.rixhub_endpoint)
        sub.set_callback(TMsg, cb)
        self.components.add(sub)
        return sub

    @staticmethod
    def _getMachineID() -> int | None:
        return 0

    @staticmethod
    def _generateID() -> int:
        return random.getrandbits(64)
