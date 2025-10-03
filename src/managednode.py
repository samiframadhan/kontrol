# managed_node.py
import zmq
import json
import threading
import time
import logging
from abc import ABC, abstractmethod
from shared_enums import NodeState, MessageType

# --- Configuration ---
ORCHESTRATOR_URL = "tcp://localhost:5559"
HEARTBEAT_INTERVAL = 2.0  # seconds
POLL_TIMEOUT = 1000 # ms

# --- Setup Logging ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class ManagedNode(ABC):
    """
    An abstract base class for a node managed by a central orchestrator.
    It handles registration, heartbeating, and state transitions.
    """
    def __init__(self, node_name: str):
        """
        Initializes the node.

        Args:
            node_name: A unique name for this node (e.g., 'camera', 'llc_interface').
        """
        self.node_name = node_name
        self.state = NodeState.UNCONFIGURED
        self.logger = logging.getLogger(self.node_name)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.DEALER)
        # Set a unique identity for the DEALER socket for routing
        self.socket.setsockopt_string(zmq.IDENTITY, self.node_name)
        self.socket.connect(ORCHESTRATOR_URL)

        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.shutdown_event = threading.Event()
        self.threads = []

    # --- Abstract methods to be implemented by child classes ---
    @abstractmethod
    def on_configure(self) -> bool:
        pass

    @abstractmethod
    def on_activate(self) -> bool:
        pass

    @abstractmethod
    def on_deactivate(self) -> bool:
        pass

    @abstractmethod
    def on_shutdown(self) -> bool:
        pass

    # --- Core Lifecycle Management Logic ---
    def _send_message(self, msg_type: MessageType, data: dict = None):
        """Helper to send a JSON message to the orchestrator."""
        message = {"type": msg_type.value, "node_name": self.node_name}
        if data:
            message.update(data)
        try:
            self.socket.send_json(message, flags=zmq.DONTWAIT)
        except zmq.Again:
            self.logger.warning("Could not send message to orchestrator, socket busy.")

    def _update_state(self, new_state: NodeState):
        """Updates the node's state and notifies the orchestrator."""
        if self.state != new_state:
            self.state = new_state
            self.logger.info(f"Transitioned to state: {self.state.value}")
            self._send_message(MessageType.STATUS_UPDATE, {"state": self.state.value})

    def _handle_command(self, command_msg: dict):
        """Processes a command received from the orchestrator."""
        command = command_msg.get("command")
        self.logger.debug(f"Received command: {command}")

        transitions = {
            "CONFIGURE": (NodeState.UNCONFIGURED, self.on_configure, NodeState.INACTIVE),
            "ACTIVATE": (NodeState.INACTIVE, self.on_activate, NodeState.ACTIVE),
            "DEACTIVATE": (NodeState.ACTIVE, self.on_deactivate, NodeState.INACTIVE),
            "SHUTDOWN": (None, self.on_shutdown, NodeState.SHUTDOWN) # Can shutdown from any state
        }

        if command in transitions:
            from_state, action, to_state = transitions[command]
            if from_state is None or self.state == from_state:
                if action():
                    self._update_state(to_state)
                else:
                    self.logger.error(f"Failed to execute action for command '{command}'")
            else:
                self.logger.warning(f"Received command '{command}' while in wrong state '{self.state.value}'")

        if self.state == NodeState.SHUTDOWN:
            self.shutdown_event.set()

    def _command_loop(self):
        """Listens for commands from the orchestrator."""
        while not self.shutdown_event.is_set():
            try:
                socks = dict(self.poller.poll(POLL_TIMEOUT))
                if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                    message = self.socket.recv_json()
                    self._handle_command(message)
            except zmq.ZMQError as e:
                self.logger.error(f"ZMQ error in command loop: {e}")
                break
            finally:
                if self.state == NodeState.SHUTDOWN:
                    self.shutdown_event.set()
        self.logger.info("Command loop terminated.")

    def _heartbeat_loop(self):
        """Sends periodic heartbeats to the orchestrator."""
        while not self.shutdown_event.is_set():
            self._send_message(MessageType.HEARTBEAT)
            self.shutdown_event.wait(HEARTBEAT_INTERVAL)
        self.logger.info("Heartbeat loop terminated.")

    def run(self):
        """The main entry point to start the node and its lifecycle management."""
        self.logger.info("Node is starting...")
        self._send_message(MessageType.REGISTER)

        cmd_thread = threading.Thread(target=self._command_loop, name=f"{self.node_name}_CmdThread", daemon=True)
        hb_thread = threading.Thread(target=self._heartbeat_loop, name=f"{self.node_name}_HbThread", daemon=True)
        self.threads.extend([cmd_thread, hb_thread])
        cmd_thread.start()
        hb_thread.start()
        self.logger.info("Lifecycle management threads started.")

        try:
            self.shutdown_event.wait()
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received.")
        finally:
            self.logger.info("Shutdown initiated...")
            self._update_state(NodeState.SHUTDOWN)
            time.sleep(0.5)
            self.shutdown_event.set()
            for thread in self.threads:
                thread.join(timeout=1.0)
            self.socket.close()
            self.context.term()
            self.logger.info("Node has been shut down successfully.")