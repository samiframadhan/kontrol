# teleop.py (Refactored with Periodic Sender Thread)
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
SPEED_STEP = 100.0
STEER_STEP = 5.0
SEND_PERIOD = 0.02  # 50 Hz send rate

# --- State Dictionary ---
# Using a dictionary to hold shared state, making it mutable and easy to pass.
state = {
    "current_speed_rpm": 0.0,
    "current_steer_angle": 0.0,
    "auto_steer_angle": 0.0,
    "is_auto_mode": False,
    "last_timestamp": time.clock_gettime_ns(time.CLOCK_MONOTONIC)
}

# --- Function Definitions (getKey, save/restore_terminal_settings) ---
# (These functions remain unchanged)
def getKey(settings, timeout=0.001):
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

# --- NEW: Periodic Sender Thread Function ---
def periodic_sender(pub_socket, state_lock, stop_event):
    """
    Runs in a separate thread to periodically send commands.
    """
    while not stop_event.is_set():
        with state_lock:
            # Safely copy the current state for sending
            speed_to_send = state["current_speed_rpm"]
            steer_to_send = state["current_steer_angle"]
            is_auto = state["is_auto_mode"]
            auto_angle = state["auto_steer_angle"]

        command_out = {
            "speed_rpm": speed_to_send,
            "steer_angle": steer_to_send
        }
        pub_socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
        pub_socket.send_json(command_out)
        
        # Update the display
        mode_str = "AUTO" if is_auto else "MANUAL"
        if is_auto:
            real_steer_angle = auto_angle / 4
            print(f"\rReal steer angle: {real_steer_angle:<3.1f}", end="")
        print(f"\rMode: {mode_str:<6} | CMD > Speed: {speed_to_send:<5.1f} RPM, Steer: {steer_to_send:<3.0f} deg", end="")

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
        args=(pub_socket, state_lock, stop_event)
    )
    sender_thread.start()
    
    # (Print statements for controls remain the same)
    print("\nControls:")
    print("  w/s: Increase/decrease speed")
    print("  a/d: Steer left/right (in MANUAL mode)")
    print("  m: Toggle between MANUAL and AUTO mode")
    print("  space: Emergency stop (set speed to 0)")
    print("  x: Reset speed and steering to 0")
    print("  q: Quit")
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
                    dt = (time.clock_gettime_ns(time.CLOCK_MONOTONIC) - state["last_timestamp"]) / 1e9
                    state["last_timestamp"] = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                    hz = 1.0 / dt if dt > 0 else 0.0
                    print(f"\rReceived auto steer angle: {received_angle:<3.1f} deg {hz:.2f}", end="")
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
                
                if key in ['w', 's', ' ', 'x'] or (not state["is_auto_mode"] and key in ['a', 'd']):
                    if key == 'w':
                        state["current_speed_rpm"] = min(state["current_speed_rpm"] + SPEED_STEP, MAX_SPEED_RPM)
                    elif key == 's':
                        state["current_speed_rpm"] = max(state["current_speed_rpm"] - SPEED_STEP, -MAX_SPEED_RPM)
                    elif key == 'a' and not state["is_auto_mode"]:
                        state["current_steer_angle"] = max(state["current_steer_angle"] - STEER_STEP, -MAX_STEER_ANGLE)
                    elif key == 'd' and not state["is_auto_mode"]:
                        state["current_steer_angle"] = min(state["current_steer_angle"] + STEER_STEP, MAX_STEER_ANGLE)
                    elif key == ' ':
                        state["current_speed_rpm"] = 0.0
                    elif key == 'x':
                        state["current_speed_rpm"] = 0.0
                        state["current_steer_angle"] = 0.0

                if state["is_auto_mode"]:
                    # In auto mode, the steering angle smoothly follows the auto angle
                    state["current_steer_angle"] = state["auto_steer_angle"]

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # --- 4. Graceful Shutdown ---
        print("\nInitiating shutdown...")
        stop_event.set() # Signal the sender thread to stop
        sender_thread.join() # Wait for the sender thread to finish

        # Send a final stop command
        stop_command = {"speed_rpm": 0.0, "steer_angle": 0.0}
        pub_socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
        pub_socket.send_json(stop_command)
        time.sleep(0.1)
        
        pub_socket.close()
        sub_socket.close()
        context.term()
        restore_terminal_settings(settings)
        print("Cleanly shut down teleop controller.")

if __name__ == "__main__":
    main()