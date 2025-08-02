# orchestrator.py (Updated for unified config and periodic state publishing)
import zmq
import time
import json
import logging
import threading
from collections import deque
from shared_enums import NodeState, MessageType
from config_mixin import ConfigMixin

# --- Setup Standard Logging ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class Orchestrator(ConfigMixin):
    """Central orchestrator for managing all nodes in the autonomous driving system."""
    
    def __init__(self, config_path="config.yaml"):
        ConfigMixin.__init__(self, config_path)
        
        # Get orchestrator-specific configuration
        orch_config = self.get_section_config('orchestrator')
        
        self.NODE_ROUTER_URL = orch_config['node_router_url']
        self.STATE_PUB_URL = orch_config['state_pub_url']
        self.COMMAND_PULL_URL = orch_config['command_pull_url']
        self.NODE_TIMEOUT = orch_config['node_timeout']
        self.POLL_TIMEOUT = orch_config['poll_timeout']
        # --- MODIFICATION: Added state publish interval from config ---
        # Get the interval for periodic state publishing, with a default of 5 seconds.
        self.STATE_PUBLISH_INTERVAL = orch_config.get('state_publish_interval', 5) 
        
        # ZMQ setup
        self.context = zmq.Context()
        self.node_socket = self.context.socket(zmq.ROUTER)
        self.node_socket.bind(self.NODE_ROUTER_URL)
        
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(self.STATE_PUB_URL)
        
        self.command_socket = self.context.socket(zmq.PULL)
        self.command_socket.bind(self.COMMAND_PULL_URL)
        
        # Poller for handling multiple sockets
        self.poller = zmq.Poller()
        self.poller.register(self.node_socket, zmq.POLLIN)
        self.poller.register(self.command_socket, zmq.POLLIN)
        
        # State management
        self.nodes = {}
        self.command_queue = deque()
        self.shutdown_event = threading.Event()

    def _log_and_publish(self, level, message):
        """Log message and publish to subscribers."""
        log_method = getattr(logging, level.lower(), logging.debug)
        log_method(message)
        log_entry = {
            "timestamp": time.strftime('%H:%M:%S'), 
            "level": level.upper(), 
            "message": message
        }
        self.pub_socket.send_multipart([b"LOG", json.dumps(log_entry).encode()])

    def _publish_state(self):
        """Publish current state of all nodes."""
        serializable_nodes = {
            identity.decode('utf-8'): {
                'name': data['name'],
                'state': data['state'].value,
                'last_heartbeat': data['last_heartbeat']
            } for identity, data in self.nodes.items()
        }
        self.pub_socket.send_multipart([b"STATE", json.dumps(serializable_nodes).encode()])
        self._log_and_publish('debug', "Published node states.") # Added debug log for clarity

    def _send_command_to_node(self, node_identity, command):
        """Send a command to a specific node."""
        if node_identity in self.nodes:
            node_name = self.nodes[node_identity]['name']
            self._log_and_publish('info', f"Sending command '{command}' to node '{node_name}'")
            self.node_socket.send_multipart([
                node_identity, 
                json.dumps({"command": command}).encode()
            ])
        else:
            self._log_and_publish('warning', 
                f"Attempted to send command '{command}' to a non-existent or removed node.")

    def _handle_node_message(self, identity, message_raw):
        """Process incoming messages from nodes."""
        state_changed = False
        try:
            message = json.loads(message_raw)
            msg_type = MessageType(message.get("type"))
            node_name = message.get("node_name")

            # Update heartbeat timestamp for existing nodes
            if identity in self.nodes:
                self.nodes[identity]["last_heartbeat"] = time.time()

            if msg_type == MessageType.REGISTER:
                self._log_and_publish('info', 
                    f"Registered node '{node_name}' [{identity.decode()}]")
                self.nodes[identity] = {
                    "name": node_name, 
                    "state": NodeState.UNCONFIGURED, 
                    "last_heartbeat": time.time()
                }
                # Automatically configure new nodes
                self.command_queue.append((identity, "CONFIGURE"))
                state_changed = True

            elif msg_type == MessageType.STATUS_UPDATE:
                if identity in self.nodes:
                    new_state = NodeState(message.get("state"))
                    if self.nodes[identity]["state"] != new_state:
                        self._log_and_publish('info', 
                            f"Node '{node_name}' updated state to: {new_state.value}")
                        self.nodes[identity]["state"] = new_state
                        state_changed = True
                        
            elif msg_type == MessageType.HEARTBEAT:
                # Heartbeat already handled by timestamp update above
                pass
                
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self._log_and_publish('error', 
                f"Failed to parse node message: {message_raw}. Error: {e}")

        if state_changed:
            self._publish_state()

    def _handle_client_command(self, command_raw):
        """Process commands from TUI or other clients."""
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
                    node_command = "DEACTIVATE"
                elif command == "SHUTDOWN_NODE":
                    node_command = "SHUTDOWN"
                
                if node_command:
                    self.command_queue.append((identity_bytes, node_command))

            # Global commands
            elif command == "SHUTDOWN_ALL":
                self._log_and_publish('info', "Client requested SHUTDOWN for all nodes.")
                for identity in list(self.nodes.keys()):
                    self.command_queue.append((identity, "SHUTDOWN"))
            
            elif command == "ACTIVATE_ALL":
                self._log_and_publish('info', "Client requested ACTIVATE for all inactive nodes.")
                for identity, data in self.nodes.items():
                    if data['state'] == NodeState.INACTIVE:
                        self.command_queue.append((identity, "ACTIVATE"))
            
            elif command == "DEACTIVATE_ALL":
                self._log_and_publish('info', "Client requested DEACTIVATE for all active nodes.")
                for identity, data in self.nodes.items():
                    if data['state'] == NodeState.ACTIVE:
                        self.command_queue.append((identity, "DEACTIVATE"))
            
            elif command == "REQUEST_SHUTDOWN":
                self._log_and_publish('info', "Client requested orchestrator shutdown.")
                self.shutdown_event.set()

        except (json.JSONDecodeError, KeyError) as e:
            self._log_and_publish('error', 
                f"Failed to parse client command: {command_raw}. Error: {e}")

    def _check_for_dead_nodes(self):
        """Remove nodes that haven't sent heartbeats within the timeout period."""
        now = time.time()
        dead_nodes_ids = [
            identity for identity, data in self.nodes.items()
            if now - data['last_heartbeat'] > self.NODE_TIMEOUT
        ]
        
        if dead_nodes_ids:
            for identity in dead_nodes_ids:
                node_name = self.nodes[identity]['name']
                self._log_and_publish('warning', 
                    f"Node '{node_name}' is dead (timeout). Removing.")
                del self.nodes[identity]
            self._publish_state()

    def _process_command_queue(self):
        """Process any pending commands in the queue."""
        if self.command_queue:
            identity, command = self.command_queue.popleft()
            self._send_command_to_node(identity, command)

    def run(self):
        """Main orchestrator loop."""
        self._log_and_publish('info', f"Orchestrator listening for nodes on {self.NODE_ROUTER_URL}")
        self._log_and_publish('info', f"Publishing state updates on {self.STATE_PUB_URL}")
        self._log_and_publish('info', f"Listening for client commands on {self.COMMAND_PULL_URL}")
        
        # --- MODIFICATION: Initialize last publish time ---
        last_state_publish_time = time.time()
        
        try:
            while not self.shutdown_event.is_set():
                # Poll for incoming messages
                socks = dict(self.poller.poll(timeout=self.POLL_TIMEOUT))
                
                # Handle node messages
                if self.node_socket in socks:
                    identity, message_raw = self.node_socket.recv_multipart()
                    self._handle_node_message(identity, message_raw)
                
                # Handle client commands
                if self.command_socket in socks:
                    command_raw = self.command_socket.recv()
                    self._handle_client_command(command_raw)
                
                # Maintenance tasks
                self._check_for_dead_nodes()
                self._process_command_queue()
                
                # --- MODIFICATION: Periodically publish state ---
                current_time = time.time()
                if current_time - last_state_publish_time >= self.STATE_PUBLISH_INTERVAL:
                    self._publish_state()
                    last_state_publish_time = current_time
                
        except KeyboardInterrupt:
            self._log_and_publish('info', "\nShutdown signal received via KeyboardInterrupt.")
        finally:
            self.shutdown()

    def shutdown(self):
        """Gracefully shutdown the orchestrator."""
        self._log_and_publish('info', "Initiating orchestrator shutdown...")
        self.shutdown_event.set()
        
        # Send shutdown commands to all remaining nodes
        for identity in list(self.nodes.keys()):
            self._send_command_to_node(identity, "SHUTDOWN")
        
        # Allow time for nodes to process shutdown commands
        time.sleep(1)
        
        # Close ZMQ resources
        self.node_socket.close()
        self.pub_socket.close()
        self.command_socket.close()
        self.context.term()
        
        logging.info("Orchestrator shut down successfully.")

    def get_node_statistics(self):
        """Get statistics about managed nodes."""
        stats = {
            'total_nodes': len(self.nodes),
            'states': {}
        }
        
        for state in NodeState:
            stats['states'][state.value] = sum(
                1 for node in self.nodes.values() 
                if node['state'] == state
            )
        
        return stats

    def get_node_by_name(self, name):
        """Find a node by its name."""
        for identity, data in self.nodes.items():
            if data['name'] == name:
                return identity, data
        return None, None

def main():
    """Main function to run the orchestrator."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Autonomous Driving System Orchestrator')
    parser.add_argument('--config', type=str, default='config.yaml', 
                       help='Path to configuration file')
    parser.add_argument('--log-level', type=str, default='INFO',
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='Set logging level')
    args = parser.parse_args()
    
    # Set logging level
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # Create and run orchestrator
    try:
        orchestrator = Orchestrator(config_path=args.config)
        orchestrator.run()
    except Exception as e:
        logging.error(f"Failed to start orchestrator: {e}", exc_info=True)
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())