import threading
import socket
import select
from typing import Tuple

from rixcore.common import (
    send_message_with_opcode,
    send_message_with_opcode_no_response,
    OPCODE,
)
from rixmsg.mediator.PubInfo import PubInfo
from rixmsg.standard.UInt32 import UInt32


class Publisher:
    def __init__(self, info: PubInfo, server: socket.socket, rixhub_endpoint: Tuple[str, int] = ("127.0.0.1", 0)):
        self.shutdown_flag = False
        self.info = info
        self.server = server
        self.connections = set()
        self.connections_mutex = threading.Lock()
        self.rixhub_endpoint = rixhub_endpoint

        client = socket.create_connection(self.rixhub_endpoint)
        if not send_message_with_opcode(
            client, self.info, OPCODE.PUB_REGISTER, self.rixhub_endpoint
        ):
            self.shutdown()

    def __del__(self):
        self.shutdown()
        client = socket.create_connection(self.rixhub_endpoint)
        send_message_with_opcode_no_response(
            client, self.info, OPCODE.PUB_DEREGISTER, self.rixhub_endpoint
        )

    def ok(self) -> bool:
        return not self.shutdown_flag

    def publish(self, msg: any) -> None:
        if msg.hash() != self.info.topic_info.message_hash:
            print("Warning: Message type mismatch in publish!")
            return

        buffer = bytearray()
        msgLen = UInt32()
        msgLen.data = msg.size()
        msgLen.serialize(buffer)
        msg.serialize(buffer)
        to_remove = []
        self.connections_mutex.acquire()
        for conn in self.connections:
            try:
                sent = conn.send(buffer)
            except Exception as e:
                to_remove.append(conn)
                continue

        for conn in to_remove:
            self.connections.remove(conn)
        self.connections_mutex.release()

    def shutdown(self) -> None:
        self.shutdown_flag = True

    def _spin_once(self) -> None:
        readable, _, _ = select.select([self.server], [], [], 0.0)
        if self.server in readable:
            conn, _ = self.server.accept()
            self.connections_mutex.acquire()
            self.connections.add(conn)
            self.connections_mutex.release()
