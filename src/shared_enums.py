from enum import Enum

class NodeState(Enum):
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    SHUTDOWN = "shutdown"

class MessageType(Enum):
    REGISTER = "REGISTER"
    HEARTBEAT = "HEARTBEAT"
    STATUS_UPDATE = "STATUS_UPDATE"
    COMMAND = "COMMAND"