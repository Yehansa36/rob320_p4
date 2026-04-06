import threading
import socket
import select
from typing import Tuple

from rixcore.common import (
    send_message_with_opcode,
    send_message_with_opcode_no_response,
    OPCODE,
)
from rixmsg.standard.UInt32 import UInt32
from rixmsg.mediator.Operation import Operation
from rixmsg.mediator.SubInfo import SubInfo
from rixmsg.mediator.PubInfo import PubInfo
from rixmsg.mediator.SubNotify import SubNotify


class Subscriber:
    def __init__(
        self,
        info: SubInfo,
        server: socket.socket,
        rixhub_endpoint: Tuple[str, int] = ("127.0.0.1", 0),
    ):
        self.shutdown_flag = False
        self.info = info
        self.server = server
        self.clients = set()
        self.callback = None
        self.callback_mutex = threading.Lock()
        self.rixhub_endpoint = rixhub_endpoint

        client = socket.create_connection(self.rixhub_endpoint)
        if not send_message_with_opcode(
            client, self.info, OPCODE.SUB_REGISTER, self.rixhub_endpoint
        ):
            self.shutdown()

    def __del__(self):
        self.shutdown()
        client = socket.create_connection(self.rixhub_endpoint)
        send_message_with_opcode_no_response(
            client, self.info, OPCODE.SUB_DEREGISTER, self.rixhub_endpoint
        )

    def ok(self) -> bool:
        return not self.shutdown_flag

    def set_callback(self, TMsg, callback: callable):
        def callback_serialized(buffer: bytearray):
            msg = TMsg()
            msg.deserialize(buffer, {"offset": 0})
            callback(msg)

        self.callback = callback_serialized

    def shutdown(self) -> None:
        self.shutdown_flag = True

    def _spin_once(self) -> None:
        readable, _, _ = select.select([self.server], [], [], 0.0)
        if self.server in readable:
            conn, _ = self.server.accept()
            # Receive SubNotify message and connect to publishers
            op = Operation()
            recvSize = op.size()
            opBuffer = bytearray(recvSize)
            conn.recv_into(opBuffer, recvSize)
            op.deserialize(opBuffer, {"offset": 0})
            msgBuffer = bytearray(op.len)
            bytesRecv = conn.recv_into(memoryview(msgBuffer), op.len)

            sub_notify = SubNotify()
            sub_notify.deserialize(msgBuffer, {"offset": 0})
            for pub in sub_notify.publishers:
                client = socket.socket()
                client.setblocking(False)
                try:
                    client.connect((pub.endpoint.address, pub.endpoint.port))
                except Exception as e:
                    pass
                self.clients.add(client)

        self.callback_mutex.acquire()
        to_remove = []
        for client in self.clients:
            readable, _, _ = select.select([client], [], [], 0.0)
            if client in readable:
                try:
                    msgLen = UInt32()
                    recvSize = msgLen.size()
                    msgLenBuffer = bytearray(recvSize)
                    bytesRecv = client.recv_into(msgLenBuffer, recvSize)
                    if bytesRecv <= 0:
                        to_remove.append(client)
                        continue
                    
                    msgLen.deserialize(msgLenBuffer, {"offset": 0})
                    msgBuffer = bytearray(msgLen.data)
                    bytesRecv = 0
                    while bytesRecv < msgLen.data:
                        bytesRecv += client.recv_into(
                            memoryview(msgBuffer)[bytesRecv:], msgLen.data - bytesRecv
                        )
                    if self.callback is not None:
                        self.callback(msgBuffer)
                except TimeoutError as e:
                    continue
                except Exception as e:
                    continue
            
        for client in to_remove:
            self.clients.remove(client)

        self.callback_mutex.release()
