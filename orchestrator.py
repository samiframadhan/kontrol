# orchestrator_core.py
import zmq
import time
import json
import logging
import threading
import yaml
from collections import deque
from shared_enums import NodeState, MessageType

# --- Configuration ---
NODE_ROUTER_URL = "tcp://*:5559"
STATE_PUB_URL = "tcp://*:5560"
COMMAND_PULL_URL = "tcp://*:5561"
NODE_TIMEOUT = 10.0
POLL_TIMEOUT = 100

# --- Setup Standard Logging ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class Orchestrator:
    def __init__(self, config_path='config.yaml'):
        """
        Initializes the Orchestrator by loading ZMQ configurations from a YAML file.
        """
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)['zmq_urls']

        self.nodes_url = config['orchestrator_nodes'].replace('localhost', '*')
        self.states_url = config['orchestrator_states'].replace('localhost', '*')
        self.cmds_url = config['orchestrator_cmds'].replace('localhost', '*')

        self.context = zmq.Context()
        # Socket to listen for nodes
        self.nodes_socket = self.context.socket(zmq.REP)
        self.nodes_socket.bind(self.nodes_url)
        # Socket to publish node states
        self.states_socket = self.context.socket(zmq.PUB)
        self.states_socket.bind(self.states_url)
        # Socket for external commands
        self.cmds_socket = self.context.socket(zmq.PULL)
        self.cmds_socket.bind(self.cmds_url)
        print("Orchestrator initialized.")
        self.nodes = {}
        self.command_queue = deque()
        self.shutdown_event = threading.Event()

    def _log_and_publish(self, level, message):
        log_method = getattr(logging, level.lower(), logging.debug)
        log_method(message)
        log_entry = {"timestamp": time.strftime('%H:%M:%S'), "level": level.upper(), "message": message}
        self.pub_socket.send_multipart([b"LOG", json.dumps(log_entry).encode()])

    def _publish_state(self):
        serializable_nodes = {
            identity.decode('utf-8'): {
                'name': data['name'],
                'state': data['state'].value,
                'last_heartbeat': data['last_heartbeat']
            } for identity, data in self.nodes.items()
        }
        self.pub_socket.send_multipart([b"STATE", json.dumps(serializable_nodes).encode()])

    def _send_command_to_node(self, node_identity, command):
        if node_identity in self.nodes:
            self._log_and_publish('info', f"Sending command '{command}' to node '{self.nodes[node_identity]['name']}'")
            self.node_socket.send_multipart([node_identity, json.dumps({"command": command}).encode()])
        else:
            self._log_and_publish('warning', f"Attempted to send command '{command}' to a non-existent or removed node.")


    def _handle_node_message(self, identity, message_raw):
        state_changed = False
        try:
            message = json.loads(message_raw)
            msg_type = MessageType(message.get("type"))
            node_name = message.get("node_name")

            if identity in self.nodes:
                self.nodes[identity]["last_heartbeat"] = time.time()

            if msg_type == MessageType.REGISTER:
                self._log_and_publish('info', f"Registered node '{node_name}' [{identity.decode()}]")
                self.nodes[identity] = {"name": node_name, "state": NodeState.UNCONFIGURED, "last_heartbeat": time.time()}
                self.command_queue.append((identity, "CONFIGURE"))
                state_changed = True

            elif msg_type == MessageType.STATUS_UPDATE:
                if identity in self.nodes:
                    new_state = NodeState(message.get("state"))
                    if self.nodes[identity]["state"] != new_state:
                        self._log_and_publish('info', f"Node '{node_name}' updated state to: {new_state.value}")
                        self.nodes[identity]["state"] = new_state
                        state_changed = True
            # No action needed for heartbeats beyond updating the timestamp
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self._log_and_publish('error', f"Failed to parse node message: {message_raw}. Error: {e}")

        if state_changed:
            self._publish_state()

    def _handle_client_command(self, command_raw):
        """Processes granular commands from TUI clients."""
        try:
            cmd_data = json.loads(command_raw)
            command = cmd_data.get("command")
            identity_str = cmd_data.get("identity")
            
            # Commands that affect a single node
            if command and identity_str:
                identity_bytes = identity_str.encode('utf-8')
                node_command = None
                if command == "CONFIGURE_NODE":
                    node_command = "CONFIGURE"
                elif command == "ACTIVATE_NODE":
                    node_command = "ACTIVATE"
                elif command == "DEACTIVATE_NODE":
                    node_command = "DEACTIVATE" # Assuming node understands this
                elif command == "SHUTDOWN_NODE":
                    node_command = "SHUTDOWN"
                
                if node_command:
                    self.command_queue.append((identity_bytes, node_command))

            # Global commands
            elif command == "SHUTDOWN_ALL":
                self._log_and_publish('info', "Client requested SHUTDOWN for all nodes.")
                for identity in list(self.nodes.keys()):
                    self.command_queue.append((identity, "SHUTDOWN"))
            
            elif command == "REQUEST_SHUTDOWN":
                self._log_and_publish('info', "Client requested orchestrator shutdown.")
                self.shutdown_event.set()

        except (json.JSONDecodeError, KeyError) as e:
            self._log_and_publish('error', f"Failed to parse client command: {command_raw}. Error: {e}")

    def _check_for_dead_nodes(self):
        now = time.time()
        dead_nodes_ids = [
            identity for identity, data in self.nodes.items()
            if now - data['last_heartbeat'] > NODE_TIMEOUT
        ]
        if dead_nodes_ids:
            for identity in dead_nodes_ids:
                self._log_and_publish('warning', f"Node '{self.nodes[identity]['name']}' is dead (timeout). Removing.")
                del self.nodes[identity]
            self._publish_state()

    def run(self):
        self._log_and_publish('info', f"Orchestrator listening for nodes on {NODE_ROUTER_URL}")
        self._log_and_publish('info', f"Publishing state on {STATE_PUB_URL}")
        self._log_and_publish('info', f"Listening for client commands on {COMMAND_PULL_URL}")
        
        try:
            while not self.shutdown_event.is_set():
                socks = dict(self.poller.poll(timeout=POLL_TIMEOUT))
                if self.node_socket in socks:
                    identity, message_raw = self.node_socket.recv_multipart()
                    self._handle_node_message(identity, message_raw)
                if self.command_socket in socks:
                    self._handle_client_command(self.command_socket.recv())
                self._check_for_dead_nodes()
                if self.command_queue:
                    identity, command = self.command_queue.popleft()
                    self._send_command_to_node(identity, command)
        except KeyboardInterrupt:
            self._log_and_publish('info', "\nShutdown signal received via KeyboardInterrupt.")
        finally:
            self.shutdown()

    def shutdown(self):
        self._log_and_publish('info', "Initiating shutdown...")
        self.shutdown_event.set()
        for identity in list(self.nodes.keys()):
            self._send_command_to_node(identity, "SHUTDOWN")
        time.sleep(1)
        self.node_socket.close()
        self.pub_socket.close()
        self.command_socket.close()
        self.context.term()
        logging.info("Orchestrator shut down successfully.")

if __name__ == "__main__":
    Orchestrator().run()