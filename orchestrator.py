# orchestrator.py
import zmq
import time
import json
import logging
from collections import deque
from shared_enums import NodeState, MessageType
import threading
import curses

# --- Configuration ---
ZMQ_ROUTER_URL = "tcp://*:5559"
NODE_TIMEOUT = 5.0  # Seconds before a node is considered dead
POLL_TIMEOUT = 100  # ms

# --- Setup Logging ---
# The logging setup will be done in the CursesUI class
# to redirect logs to the curses window.

class CursesHandler(logging.Handler):
    """A logging handler that writes messages to a curses pad."""
    def __init__(self, pad, max_lines):
        super().__init__()
        self.pad = pad
        self.max_lines = max_lines
        self.lines = []

    def emit(self, record):
        """Emits a log record to the curses pad."""
        msg = self.format(record)
        self.lines.append(msg)
        if len(self.lines) > self.max_lines:
            self.lines.pop(0)
        
        self.pad.clear()
        for i, line in enumerate(self.lines):
            self.pad.addstr(i, 0, line)
        
        # Determine the visible portion of the pad
        pad_y, _ = self.pad.getmaxyx()
        visible_start_line = max(0, len(self.lines) - (pad_y - 2))
        
        pad_height, pad_width = self.pad.getmaxyx()
        screen_height, screen_width = curses.LINES - 1, curses.COLS // 2 - 2
        
        self.pad.refresh(visible_start_line, 0, curses.LINES // 2 + 1, curses.COLS // 2 + 1, curses.LINES - 2, curses.COLS - 2)

class CursesUI:
    """Manages the curses-based user interface for the orchestrator."""
    def __init__(self, stdscr, orchestrator):
        self.stdscr = stdscr
        self.orchestrator = orchestrator
        self.selected_option = 0
        self.menu_options = ["List all nodes", "Reactivate a node", "Shutdown all nodes", "Exit"]

        curses.curs_set(0)
        self.stdscr.nodelay(1)
        self.stdscr.timeout(100)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)

        self.setup_windows()
        self.setup_logging()

    def setup_windows(self):
        """Sets up the individual windows (panes) for the UI."""
        h, w = self.stdscr.getmaxyx()
        
        # Node window
        self.node_win = curses.newwin(h, w // 2, 0, 0)
        self.node_win.box()
        self.node_win.addstr(0, 2, " Nodes ", curses.A_BOLD)
        
        # Menu window
        self.menu_win = curses.newwin(h // 2, w // 2, 0, w // 2)
        self.menu_win.box()
        self.menu_win.addstr(0, 2, " Menu ", curses.A_BOLD)

        # Log window and pad
        self.log_win = curses.newwin(h // 2, w // 2, h // 2, w // 2)
        self.log_win.box()
        self.log_win.addstr(0, 2, " Logs ", curses.A_BOLD)
        self.log_pad = curses.newpad(1000, w // 2 - 2)
        self.log_pad.scrollok(True)

    def setup_logging(self):
        """Configures logging to use the CursesHandler."""
        log_handler = CursesHandler(self.log_pad, 1000)
        log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%H:%M:%S')
        log_handler.setFormatter(log_formatter)
        
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        logger.addHandler(log_handler)

    def draw_nodes(self):
        """Draws the list of connected nodes."""
        self.node_win.clear()
        self.node_win.box()
        self.node_win.addstr(0, 2, " Nodes ", curses.A_BOLD)
        
        if not self.orchestrator.nodes:
            self.node_win.addstr(2, 2, "No nodes connected.")
        else:
            for i, (identity, data) in enumerate(self.orchestrator.nodes.items()):
                state_color = curses.color_pair(2) if data['state'] == NodeState.ACTIVE else \
                              curses.color_pair(3) if data['state'] == NodeState.INACTIVE else \
                              curses.color_pair(1)
                self.node_win.addstr(i + 2, 2, f"- {data['name']} (State: ", state_color)
                self.node_win.addstr(f"{data['state'].value}", state_color | curses.A_BOLD)
                self.node_win.addstr(")")
        self.node_win.refresh()

    def draw_menu(self):
        """Draws the interactive menu."""
        self.menu_win.clear()
        self.menu_win.box()
        self.menu_win.addstr(0, 2, " Menu ", curses.A_BOLD)
        
        for i, option in enumerate(self.menu_options):
            if i == self.selected_option:
                self.menu_win.addstr(i + 2, 2, f"> {option}", curses.A_REVERSE)
            else:
                self.menu_win.addstr(i + 2, 2, f"  {option}")
        self.menu_win.refresh()

    def handle_input(self):
        """Handles user input for menu navigation and selection."""
        key = self.stdscr.getch()
        if key == curses.KEY_UP:
            self.selected_option = (self.selected_option - 1) % len(self.menu_options)
        elif key == curses.KEY_DOWN:
            self.selected_option = (self.selected_option + 1) % len(self.menu_options)
        elif key == ord('\n'):
            self.execute_option()

    def execute_option(self):
        """Executes the action associated with the selected menu option."""
        option = self.menu_options[self.selected_option]
        if option == "Reactivate a node":
            self.reactivate_node_menu()
        elif option == "Shutdown all nodes":
            logging.info("Queuing SHUTDOWN command for all nodes.")
            for identity in list(self.orchestrator.nodes.keys()):
                self.orchestrator.command_queue.append((identity, "SHUTDOWN"))
        elif option == "Exit":
            self.orchestrator.shutdown_event.set()

    def reactivate_node_menu(self):
        """Presents a sub-menu to select an inactive node to reactivate."""
        inactive_nodes = {
            i: (identity, data['name'])
            for i, (identity, data) in enumerate(self.orchestrator.nodes.items())
            if data['state'] == NodeState.INACTIVE
        }

        if not inactive_nodes:
            logging.info("No inactive nodes to reactivate.")
            return

        selected_node = 0
        while True:
            self.menu_win.clear()
            self.menu_win.box()
            self.menu_win.addstr(0, 2, " Reactivate Node ", curses.A_BOLD)
            
            for i, (idx, (_, name)) in enumerate(inactive_nodes.items()):
                if i == selected_node:
                    self.menu_win.addstr(i + 2, 2, f"> {name}", curses.A_REVERSE)
                else:
                    self.menu_win.addstr(i + 2, 2, f"  {name}")
            self.menu_win.refresh()

            key = self.stdscr.getch()
            if key == curses.KEY_UP:
                selected_node = (selected_node - 1) % len(inactive_nodes)
            elif key == curses.KEY_DOWN:
                selected_node = (selected_node + 1) % len(inactive_nodes)
            elif key == ord('\n'):
                node_to_activate = list(inactive_nodes.values())[selected_node]
                identity, name = node_to_activate
                self.orchestrator.command_queue.append((identity, "ACTIVATE"))
                logging.info(f"Queued ACTIVATE command for {name}.")
                break
            elif key == 27: # ESC key
                break


    def refresh_all(self):
        """Refreshes all UI components."""
        self.stdscr.refresh()
        self.draw_nodes()
        self.draw_menu()
        self.log_win.refresh()
        
        # Refresh the visible part of the log pad
        pad_y, _ = self.log_pad.getmaxyx()
        pad_height, pad_width = self.log_pad.getmaxyx()
        screen_height, screen_width = curses.LINES // 2 - 2, curses.COLS // 2 - 2
        visible_start_line = max(0, len(self.log_pad.getbegyx()) - (screen_height))
        
        self.log_pad.refresh(0, 0, curses.LINES // 2 + 1, curses.COLS // 2 + 1, curses.LINES - 2, curses.COLS - 2)


class Orchestrator:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.bind(ZMQ_ROUTER_URL)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.nodes = {}  # {identity: {name, state, last_heartbeat}}
        self.command_queue = deque()
        self.shutdown_event = threading.Event()

    def _send_command(self, node_identity, command):
        """Sends a command to a specific node."""
        logging.info(f"Sending command '{command}' to node '{self.nodes[node_identity]['name']}'")
        message = {"command": command}
        self.socket.send_multipart([node_identity, json.dumps(message).encode()])

    def _handle_incoming_message(self, identity, message_raw):
        """Processes messages received from nodes."""
        try:
            message = json.loads(message_raw)
            msg_type = MessageType(message.get("type"))
            node_name = message.get("node_name")

            if identity in self.nodes:
                self.nodes[identity]["last_heartbeat"] = time.time()

            if msg_type == MessageType.REGISTER:
                logging.info(f"Registered node '{node_name}' [{identity.decode()}]")
                self.nodes[identity] = {
                    "name": node_name,
                    "state": NodeState.UNCONFIGURED,
                    "last_heartbeat": time.time()
                }
                self.command_queue.append((identity, "CONFIGURE"))

            elif msg_type == MessageType.STATUS_UPDATE:
                if identity in self.nodes:
                    new_state = NodeState(message.get("state"))
                    logging.info(f"Node '{node_name}' updated state to: {new_state.value}")
                    self.nodes[identity]["state"] = new_state

            elif msg_type == MessageType.HEARTBEAT:
                logging.debug(f"Heartbeat from '{node_name}'")

        except (json.JSONDecodeError, KeyError, ValueError) as e:
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

    def run(self, stdscr):
        """Main orchestrator loop with curses UI."""
        self.ui = CursesUI(stdscr, self)
        logging.info(f"Orchestrator listening on {ZMQ_ROUTER_URL}")
        
        try:
            while not self.shutdown_event.is_set():
                self.ui.handle_input()
                self.ui.refresh_all()

                socks = dict(self.poller.poll(timeout=POLL_TIMEOUT))
                if self.socket in socks:
                    identity, message_raw = self.socket.recv_multipart()
                    self._handle_incoming_message(identity, message_raw)

                self._check_for_dead_nodes()

                if self.command_queue:
                    identity, command = self.command_queue.popleft()
                    if identity in self.nodes:
                        self._send_command(identity, command)
        except KeyboardInterrupt:
            logging.info("\nShutdown signal received.")
        finally:
            self.shutdown_event.set()
            logging.info("Notifying all nodes to shutdown.")
            for identity in list(self.nodes.keys()):
                self._send_command(identity, "SHUTDOWN")
            time.sleep(1)

            self.socket.close()
            self.context.term()
            logging.info("Orchestrator shut down.")

def main(stdscr):
    orchestrator = Orchestrator()
    orchestrator.run(stdscr)

if __name__ == "__main__":
    curses.wrapper(main)