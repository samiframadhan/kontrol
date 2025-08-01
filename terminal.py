# tui_client.py
import zmq
import json
import logging
import curses
from shared_enums import NodeState

# --- Configuration ---
STATE_SUB_URL = "tcp://localhost:5560"
COMMAND_PUSH_URL = "tcp://localhost:5561"

# (CursesHandler class remains the same as previous version)
class CursesHandler(logging.Handler):
    def __init__(self, pad, max_lines):
        super().__init__()
        self.pad = pad
        self.max_lines = max_lines
        self.lines = []
    def emit(self, record):
        msg = self.format(record)
        self.lines.append(msg)
        if len(self.lines) > self.max_lines: self.lines.pop(0)
        self.pad.clear()
        h, w = self.pad.getmaxyx()
        for i, line in enumerate(self.lines): self.pad.addstr(i, 0, line[:w-1])
        pad_y_pos = max(0, len(self.lines) - (curses.LINES // 2 - 2))
        self.pad.refresh(pad_y_pos, 0, curses.LINES // 2 + 1, curses.COLS // 2 + 1, curses.LINES - 2, curses.COLS - 2)

class TUIClient:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.running = True
        self.nodes = {}
        self.selected_option = 0
        self.menu_options = ["Manage Nodes", "Shutdown all nodes", "Exit TUI", "Shutdown Orchestrator & Exit"]
        
        self.context = zmq.Context()
        self.command_socket = self.context.socket(zmq.PUSH)
        self.command_socket.connect(COMMAND_PUSH_URL)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(STATE_SUB_URL)
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, b"STATE")
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, b"LOG")
        self.poller = zmq.Poller()
        self.poller.register(self.sub_socket, zmq.POLLIN)

        self._setup_curses()
        self._setup_logging()

    def _setup_curses(self):
        curses.curs_set(0)
        self.stdscr.nodelay(1)
        self.stdscr.timeout(100)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)
        h, w = self.stdscr.getmaxyx()
        self.node_win = curses.newwin(h, w // 2, 0, 0)
        self.menu_win = curses.newwin(h // 2, w // 2, 0, w // 2)
        self.log_win = curses.newwin(h // 2, w // 2, h // 2, w // 2)
        self.log_pad = curses.newpad(1000, w // 2 - 2)

    def _setup_logging(self):
        log_handler = CursesHandler(self.log_pad, 1000)
        log_formatter = logging.Formatter('%(message)s')
        log_handler.setFormatter(log_formatter)
        self.logger = logging.getLogger('TUI_Logger')
        self.logger.setLevel(logging.INFO)
        self.logger.addHandler(log_handler)

    def _send_command(self, command_data):
        self.command_socket.send_json(command_data)

    def draw_nodes(self, selected_nodes=None, cursor_pos=0):
        self.node_win.clear()
        self.node_win.box()
        self.node_win.addstr(0, 2, " Nodes ", curses.A_BOLD)
        
        if not self.nodes:
            self.node_win.addstr(2, 2, "No nodes reported.")
        else:
            sorted_nodes = sorted(self.nodes.items(), key=lambda item: item[1]['name'])
            for i, (identity, data) in enumerate(sorted_nodes):
                state_val = data['state']
                state_color = curses.color_pair(2) if state_val == NodeState.ACTIVE.value else \
                              curses.color_pair(3) if state_val == NodeState.INACTIVE.value else \
                              curses.color_pair(1)
                
                prefix = "  "
                if selected_nodes is not None:
                    prefix = "[*]" if identity in selected_nodes else "[ ]"
                
                line_attr = curses.A_REVERSE if selected_nodes is not None and i == cursor_pos else curses.A_NORMAL
                
                self.node_win.addstr(i + 2, 2, f"{prefix} {data['name']} ({state_val})", state_color | line_attr)

        self.node_win.refresh()

    def _execute_option(self):
        option = self.menu_options[self.selected_option]
        if option == "Manage Nodes":
            self._manage_nodes_menu()
        elif option == "Shutdown all nodes":
            self._send_command({"command": "SHUTDOWN_ALL"})
            self.logger.info("Command sent: SHUTDOWN_ALL")
        elif option == "Exit TUI":
            self.running = False
        elif option == "Shutdown Orchestrator & Exit":
            self._send_command({"command": "REQUEST_SHUTDOWN"})
            self.running = False

    def _manage_nodes_menu(self):
        selected_identities = set()
        cursor_pos = 0
        sorted_nodes = sorted(self.nodes.items(), key=lambda item: item[1]['name'])
        
        while True:
            self.draw_nodes(selected_nodes=selected_identities, cursor_pos=cursor_pos)
            self.menu_win.clear(); self.menu_win.box()
            self.menu_win.addstr(0, 2, " Manage Nodes ", curses.A_BOLD)
            self.menu_win.addstr(2, 2, "Space: Select | a: Select All")
            self.menu_win.addstr(3, 2, "Enter: Actions | Esc: Back")
            self.menu_win.refresh()
            
            key = self.stdscr.getch()
            if not sorted_nodes:
                if key != -1: break
                else: continue
            
            if key == curses.KEY_UP:
                cursor_pos = (cursor_pos - 1) % len(sorted_nodes)
            elif key == curses.KEY_DOWN:
                cursor_pos = (cursor_pos + 1) % len(sorted_nodes)
            elif key == ord(' '):
                identity = sorted_nodes[cursor_pos][0]
                if identity in selected_identities: selected_identities.remove(identity)
                else: selected_identities.add(identity)
            elif key == ord('a'):
                if len(selected_identities) == len(sorted_nodes): selected_identities.clear()
                else: selected_identities.update(self.nodes.keys())
            elif key == ord('\n') and selected_identities:
                self._show_action_menu(selected_identities)
                break
            elif key == 27: # ESC
                break
    
    def _show_action_menu(self, identities):
        actions = []
        if len(identities) == 1:
            identity = list(identities)[0]
            state = self.nodes[identity]['state']
            if state == NodeState.UNCONFIGURED.value: actions.append("Configure")
            elif state == NodeState.INACTIVE.value: actions.append("Activate")
            elif state == NodeState.ACTIVE.value: actions.append("Deactivate")
        else: # Multiple nodes selected
            actions = ["Configure", "Activate", "Deactivate"]
        
        actions.append("Shutdown")
        actions.append("Cancel")
        
        selected_action_idx = 0
        while True:
            self.menu_win.clear(); self.menu_win.box()
            self.menu_win.addstr(0, 2, " Select Action ", curses.A_BOLD)
            for i, action in enumerate(actions):
                attr = curses.A_REVERSE if i == selected_action_idx else curses.A_NORMAL
                self.menu_win.addstr(i + 2, 2, f"> {action}", attr)
            self.menu_win.refresh()
            
            key = self.stdscr.getch()
            if key == curses.KEY_UP: selected_action_idx = (selected_action_idx - 1) % len(actions)
            elif key == curses.KEY_DOWN: selected_action_idx = (selected_action_idx + 1) % len(actions)
            elif key == 27 or (key == ord('\n') and actions[selected_action_idx] == "Cancel"): break
            elif key == ord('\n'):
                action_str = actions[selected_action_idx]
                command_suffix = "_NODE"
                for identity in identities:
                    self._send_command({"command": f"{action_str.upper()}{command_suffix}", "identity": identity})
                self.logger.info(f"Sent '{action_str}' command for {len(identities)} node(s).")
                break

    def handle_subscription(self):
        socks = dict(self.poller.poll(timeout=10))
        if self.sub_socket in socks:
            topic, payload = self.sub_socket.recv_multipart()
            if topic == b"STATE": self.nodes = json.loads(payload)
            elif topic == b"LOG":
                log_entry = json.loads(payload)
                self.logger.info(f"{log_entry['timestamp']} - {log_entry['level']} - {log_entry['message']}")

    def run(self):
        # Main UI loop
        while self.running:
            self.handle_input()
            self.handle_subscription()
            self.stdscr.refresh()
            self.draw_nodes()
            self.draw_menu()
            self.log_win.box()
            self.log_win.addstr(0, 2, " Logs ", curses.A_BOLD)
            self.log_win.refresh()
            if self.logger.handlers:
                self.logger.handlers[0].emit(logging.LogRecord("", logging.INFO, "", 0, "", (), None))
        
        # Cleanup
        self.command_socket.close()
        self.sub_socket.close()
        self.context.term()

    # Methods draw_menu and handle_input are adjusted to call the new menu flows
    def draw_menu(self):
        self.menu_win.clear()
        self.menu_win.box()
        self.menu_win.addstr(0, 2, " Menu ", curses.A_BOLD)
        for i, option in enumerate(self.menu_options):
            attr = curses.A_REVERSE if i == self.selected_option else curses.A_NORMAL
            self.menu_win.addstr(i + 2, 2, f"> {option}", attr)
        self.menu_win.refresh()

    def handle_input(self):
        key = self.stdscr.getch()
        if key == curses.KEY_UP:
            self.selected_option = (self.selected_option - 1) % len(self.menu_options)
        elif key == curses.KEY_DOWN:
            self.selected_option = (self.selected_option + 1) % len(self.menu_options)
        elif key == ord('\n'):
            self._execute_option()

def main(stdscr):
    TUIClient(stdscr).run()

if __name__ == "__main__":
    # Ensure shared_enums.py is in the same directory
    curses.wrapper(main)