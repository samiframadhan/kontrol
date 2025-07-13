# orchestrator.py
import zmq
import time
import json
import logging
from collections import deque

# --- Configuration ---
ZMQ_ROUTER_URL = "tcp://*:5559"
NODE_TIMEOUT = 5.0  # Seconds before a node is considered dead
POLL_TIMEOUT = 1000 # ms

# --- Setup Logging ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - Orchestrator - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class Orchestrator:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.bind(ZMQ_ROUTER_URL)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.nodes = {}  # {identity: {name, state, last_heartbeat}}
        self.command_queue = deque()

    def _send_command(self, node_identity, command):
        """Sends a command to a specific node."""
        logging.info(f"Sending command '{command}' to node '{self.nodes[node_identity]['name']}'")
        message = {"command": command}
        self.socket.send_multipart([node_identity, json.dumps(message).encode()])

    def _handle_incoming_message(self, identity, message_raw):
        """Processes messages received from nodes."""
        try:
            message = json.loads(message_raw)
            msg_type = message.get("type")
            node_name = message.get("node_name")

            # Update heartbeat time for any message from a known node
            if identity in self.nodes:
                self.nodes[identity]["last_heartbeat"] = time.time()

            if msg_type == "REGISTER":
                logging.info(f"Registered node '{node_name}' [{identity.decode()}]")
                self.nodes[identity] = {
                    "name": node_name,
                    "state": "unconfigured",
                    "last_heartbeat": time.time()
                }
                # Automatically queue a CONFIGURE command upon registration
                self.command_queue.append((identity, "CONFIGURE"))

            elif msg_type == "STATUS_UPDATE":
                if identity in self.nodes:
                    new_state = message.get("state")
                    logging.info(f"Node '{node_name}' updated state to: {new_state}")
                    self.nodes[identity]["state"] = new_state
                    # Simple state machine logic: if a node becomes inactive, activate it.
                    if new_state == "inactive":
                        self.command_queue.append((identity, "ACTIVATE"))

            elif msg_type == "HEARTBEAT":
                logging.debug(f"Heartbeat from '{node_name}'")

        except (json.JSONDecodeError, KeyError) as e:
            logging.error(f"Failed to parse message: {message_raw}. Error: {e}")

    def _check_for_dead_nodes(self):
        """Periodically checks for and removes timed-out nodes."""
        now = time.time()
        dead_nodes = [
            identity for identity, data in self.nodes.items()
            if now - data['last_heartbeat'] > NODE_TIMEOUT
        ]
        for identity in dead_nodes:
            logging.warning(f"Node '{self.nodes[identity]['name']}' is dead (timeout). Removing.")
            del self.nodes[identity]

    def run(self):
        """Main orchestrator loop."""
        logging.info(f"Orchestrator listening on {ZMQ_ROUTER_URL}")
        try:
            while True:
                # 1. Poll for incoming messages
                socks = dict(self.poller.poll(timeout=POLL_TIMEOUT))
                if self.socket in socks:
                    identity, message_raw = self.socket.recv_multipart()
                    self._handle_incoming_message(identity, message_raw)

                # 2. Check for dead nodes
                self._check_for_dead_nodes()

                # 3. Process the command queue
                if self.command_queue:
                    identity, command = self.command_queue.popleft()
                    if identity in self.nodes: # Ensure node hasn't died
                        self._send_command(identity, command)

        except KeyboardInterrupt:
            logging.info("\nShutdown signal received. Notifying all nodes.")
            for identity in list(self.nodes.keys()):
                self._send_command(identity, "SHUTDOWN")
            # Give nodes a moment to process shutdown
            time.sleep(1)
        finally:
            self.socket.close()
            self.context.term()
            logging.info("Orchestrator shut down.")

if __name__ == "__main__":
    orchestrator = Orchestrator()
    orchestrator.run()
