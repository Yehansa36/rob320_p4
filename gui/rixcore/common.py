import socket
from enum import IntEnum
from typing import Tuple

from rixmsg.mediator.Operation import Operation
from rixmsg.mediator.Status import Status

RIXHUB_PORT = 48104
MACHINE_ID_FILE = "~/.rix/machine_id"

class OPCODE(IntEnum):
    NODE_REGISTER = 80
    SUB_REGISTER = 81
    PUB_REGISTER = 82
    SRV_REGISTER = 83
    ACT_REGISTER = 84
    SUB_NOTIFY = 90
    PUB_NOTIFY = 91
    NODE_DEREGISTER = 100
    SUB_DEREGISTER = 101
    PUB_DEREGISTER = 102
    SRV_DEREGISTER = 103
    ACT_DEREGISTER = 104
    SRV_REQUEST = 140
    ACT_REQUEST = 141
    SRV_RESPONSE = 150
    ACT_RESPONSE = 151
    TERMINATE = 160


def send_message_with_opcode_no_response(
    client: socket.socket, msg: any, opcode: int, endpoint: Tuple[str, int]
) -> bool:
    op = Operation()
    op.opcode = opcode
    op.len = msg.size()
    buffer = bytearray()
    op.serialize(buffer)
    msg.serialize(buffer)
    try:
        sent = client.send(buffer)
    except Exception as e:
        return False
    return True


def send_message_with_opcode(
    client: socket.socket, msg: any, opcode: int, endpoint: Tuple[str, int]
) -> bool:
    if not send_message_with_opcode_no_response(client, msg, opcode, endpoint):
        return False

    status = Status()
    msgLen = status.size()
    msgBuffer = bytearray(msgLen)
    client.recv_into(memoryview(msgBuffer), msgLen)
    status.deserialize(msgBuffer, {"offset": 0})
    if status.error != 0:
        return False
    return True