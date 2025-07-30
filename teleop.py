# teleop.py (Refactored with Periodic Sender Thread and Gradual Brake)
import sys
import time
import json
import zmq
import threading
import steering_command_pb2  # Import your Protobuf message definition

# Platform-specific imports for keyboard
try:
    import tty
    import termios
    import select
    UNIX = True
except ImportError:
    import msvcrt
    UNIX = False

# --- ZMQ Configuration ---
ZMQ_PUB_URL = "ipc:///tmp/llc.ipc"         # For publishing final commands
ZMQ_SUB_URL = "ipc:///tmp/teleop_cmd.ipc"        # For subscribing to lane assist angles
LANE_ASSIST_TOPIC = "lane_assist_angle"

# --- Control Settings ---
MAX_SPEED_RPM = 1500.0
MAX_STEER_ANGLE = 120.0
MAX_BRAKE_FORCE = 100.0  # New: Maximum brake force percentage
SPEED_STEP = 100.0
STEER_STEP = 5.0
BRAKE_STEP = 2.0     # New: How quickly the brake applies when speed is 0
SEND_PERIOD = 0.02  # 50 Hz send rate

# --- State Dictionary ---
# Using a dictionary to hold shared state, making it mutable and easy to pass.
state = {
    "current_speed_rpm": 0.0,
    "current_steer_angle": 0.0,
    "brake_force": 0.0,  # New: Added brake force to the state
    "auto_steer_angle": 0.0,
    "is_auto_mode": False,
    "last_timestamp": time.clock_gettime_ns(time.CLOCK_MONOTONIC)
}

# --- Function Definitions (getKey, save/restore_terminal_settings) ---
# (These functions remain unchanged)
def getKey(settings, timeout=0.01):
    """Gets a single character from standard input."""
    if UNIX:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    else:
        # A small sleep is needed for Windows to prevent high CPU usage
        time.sleep(timeout)
        if msvcrt.kbhit():
            key = msvcrt.getch().decode('utf-8')
        else:
            key = ''
    return key

def save_terminal_settings():
    """Saves the current terminal settings."""
    if UNIX:
        return termios.tcgetattr(sys.stdin)
    return None

def restore_terminal_settings(old_settings):
    """Restores terminal settings to their original state."""
    if UNIX:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# --- MODIFIED: Periodic Sender Thread Function ---
def periodic_sender(pub_socket, state_lock, stop_event):
    """
    Runs in a separate thread to periodically send commands.
    """
    while not stop_event.is_set():
        with state_lock:
            # Safely copy the current state for sending
            speed_to_send = state["current_speed_rpm"]
            steer_to_send = state["current_steer_angle"]
            brake_to_send = state["brake_force"]  # New
            is_auto = state["is_auto_mode"]
            auto_angle = state["auto_steer_angle"]

        command_out = {
            "speed_rpm": speed_to_send,
            "steer_angle": steer_to_send,
            "brake_force": brake_to_send  # New: Add brake force to payload
        }
        pub_socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
        pub_socket.send_json(command_out)
        
        # Update the display
        mode_str = "AUTO" if is_auto else "MANUAL"
        display_str = (
            f"\rMode: {mode_str:<6} | "
            f"CMD > Speed: {speed_to_send:<5.1f} RPM, "
            f"Steer: {steer_to_send:<3.0f} deg, "
            f"Brake: {brake_to_send:<3.0f}%"
        )
        # In auto mode, add lane assist info
        if is_auto:
            real_steer_angle = auto_angle / 4
            display_str += f" | Auto Steer Angle: {real_steer_angle:<3.1f}"
        
        print(display_str, end=" " * 5) # Whitespace to clear previous line

        time.sleep(SEND_PERIOD)

def main():
    settings = save_terminal_settings()

    # --- ZMQ Setup ---
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(ZMQ_PUB_URL)
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(ZMQ_SUB_URL)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, LANE_ASSIST_TOPIC)
    
    print(f"Publishing final commands on {ZMQ_PUB_URL}")
    print(f"Subscribing to lane assist commands from {ZMQ_SUB_URL}")

    # --- Threading Setup ---
    state_lock = threading.Lock()
    stop_event = threading.Event()
    sender_thread = threading.Thread(
        target=periodic_sender,
        args=(pub_socket, state_lock, stop_event),
        name="PeriodicSenderThread"
    )
    sender_thread.start()
    
    # (Print statements for controls remain the same)
    print("\nControls:")
    print("  w/s:   Increase/decrease speed")
    print("  a/d:   Steer left/right (in MANUAL mode)")
    print("  m:     Toggle between MANUAL and AUTO mode")
    print("  space: Emergency stop (set speed to 0, engage gradual brake)")
    print("  x:     Reset speed, steering, and brake to 0")
    print("  q:     Quit")
    print("-------------------------------------------")

    try:
        while True:
            # --- 1. Check for incoming autonomous commands (non-blocking) ---
            try:
                topic, serialized_data = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                command = steering_command_pb2.SteeringCommand()
                command.ParseFromString(serialized_data)
                received_angle = command.auto_steer_angle
                
                with state_lock:
                    state["auto_steer_angle"] = received_angle * -4.0

            except zmq.Again:
                pass # No message received, just continue

            # --- 2. Get keyboard input ---
            key = getKey(settings)
            if key == 'q':
                print("\nExiting program...")
                break

            # --- 3. Update state based on input ---
            with state_lock:
                if key == 'm':
                    state["is_auto_mode"] = not state["is_auto_mode"]
                    mode_str = "AUTO" if state["is_auto_mode"] else "MANUAL"
                    print(f"\n--- Switched to {mode_str} mode ---")

                # If user intends to move, disengage the brake
                if key in ['w', 's']:
                    state["brake_force"] = 0.0
                
                # Handle all other key presses
                if key == 'w':
                    state["current_speed_rpm"] = min(state["current_speed_rpm"] + SPEED_STEP, MAX_SPEED_RPM)
                elif key == 's':
                    state["current_speed_rpm"] = max(state["current_speed_rpm"] - SPEED_STEP, -MAX_SPEED_RPM)
                elif key == 'a' and not state["is_auto_mode"]:
                    state["current_steer_angle"] = max(state["current_steer_angle"] - STEER_STEP, -MAX_STEER_ANGLE)
                elif key == 'd' and not state["is_auto_mode"]:
                    state["current_steer_angle"] = min(state["current_steer_angle"] + STEER_STEP, MAX_STEER_ANGLE)
                elif key == ' ':
                    state["current_speed_rpm"] = 0.0 # Setting speed to 0 will trigger the brake logic
                elif key == 'x':
                    state["current_speed_rpm"] = 0.0
                    state["current_steer_angle"] = 0.0
                    state["brake_force"] = 0.0 # Explicitly reset brake to 0

                # --- New: Gradual Brake Application Logic ---
                if state["current_speed_rpm"] == 0.0:
                    # If speed is zero, gradually increase brake force up to the max
                    if state["brake_force"] < MAX_BRAKE_FORCE:
                        state["brake_force"] = min(state["brake_force"] + BRAKE_STEP, MAX_BRAKE_FORCE)
                
                # In auto mode, the steering angle smoothly follows the auto angle
                if state["is_auto_mode"]:
                    state["current_steer_angle"] = state["auto_steer_angle"]
            
            # The main loop sleep is handled by the non-blocking getKey timeout

    except (KeyboardInterrupt, SystemExit):
        print("\nCtrl+C detected. Exiting...")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # --- 4. Graceful Shutdown ---
        print("\nInitiating shutdown...")
        stop_event.set() # Signal the sender thread to stop
        sender_thread.join() # Wait for the sender thread to finish

        # Send a final stop command with full brakes
        stop_command = {"speed_rpm": 0.0, "steer_angle": 0.0, "brake_force": 100.0}
        pub_socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
        pub_socket.send_json(stop_command)
        time.sleep(0.1) # Allow time for the message to be sent
        
        pub_socket.close()
        sub_socket.close()
        context.term()
        restore_terminal_settings(settings)
        print("Cleanly shut down teleop controller.")

if __name__ == "__main__":
    main()