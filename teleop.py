# teleop.py (Integrated Version)
import sys
import time
import json
import zmq
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

# --- Function Definitions (getKey, save/restore_terminal_settings) ---
# (These functions remain unchanged from your original file)
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

def main():
    settings = save_terminal_settings()

    # --- ZMQ Setup (no changes here) ---
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(ZMQ_PUB_URL)
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(ZMQ_SUB_URL)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, LANE_ASSIST_TOPIC)
    
    print(f"Publishing final commands on {ZMQ_PUB_URL}")
    print(f"Subscribing to lane assist commands from {ZMQ_SUB_URL}")

    # --- State Variables (no changes here) ---
    current_speed_rpm = 0.0
    current_steer_angle = 0.0
    auto_steer_angle = 0.0
    is_auto_mode = False
    last_timestamp = time.clock_gettime_ns(time.CLOCK_MONOTONIC)

    # (Print statements for controls remain the same)

    try:
        while True:
            # --- 1. Check for incoming autonomous commands (non-blocking) ---
            # --- MODIFICATION START ---
            try:
                # Receive a multipart message: topic and serialized binary data
                topic, serialized_data = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                
                # Create a Protobuf message instance and parse the data into it
                command = steering_command_pb2.SteeringCommand()
                command.ParseFromString(serialized_data)
                
                # Access the data using the generated property
                received_angle = command.auto_steer_angle
                dt = (time.clock_gettime_ns(time.CLOCK_MONOTONIC) - last_timestamp) / 1e9
                last_timestamp = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                hz = 1.0 / dt if dt > 0 else 0.0
                print(f"\rReceived auto steer angle: {received_angle:<3.1f} deg {hz:.2f}", end="")
                
                # Your original logic
                auto_steer_angle = received_angle * -4.0

            except zmq.Again:
                # No message received, just continue
                pass
            # --- MODIFICATION END ---


            # --- The rest of the loop (steps 2, 3, 4, 5) remains identical. ---
            # --- No other changes are needed in the rest of the file. ---
            
            key = getKey(settings)
            changed = False
            
            if key == 'm':
                is_auto_mode = not is_auto_mode
                mode_str = "AUTO" if is_auto_mode else "MANUAL"
                print(f"\n--- Switched to {mode_str} mode ---")
                changed = True
            
            if key in ['w', 's', ' ', 'x'] or (not is_auto_mode and key in ['a', 'd']):
                changed = True
                if key == 'w':
                    current_speed_rpm = min(current_speed_rpm + SPEED_STEP, MAX_SPEED_RPM)
                elif key == 's':
                    current_speed_rpm = max(current_speed_rpm - SPEED_STEP, -MAX_SPEED_RPM)
                elif key == 'a' and not is_auto_mode:
                    current_steer_angle = max(current_steer_angle - STEER_STEP, -MAX_STEER_ANGLE)
                elif key == 'd' and not is_auto_mode:
                    current_steer_angle = min(current_steer_angle + STEER_STEP, MAX_STEER_ANGLE)
                elif key == ' ':
                    current_speed_rpm = 0.0
                elif key == 'x':
                    current_speed_rpm = 0.0
                    current_steer_angle = 0.0
            elif key == 'q':
                print("\nExiting program...")
                break

            if is_auto_mode:
                if abs(current_steer_angle - auto_steer_angle) > 0.1:
                     current_steer_angle = auto_steer_angle
                     changed = True

            if changed:
                command_out = {
                    "speed_rpm": current_speed_rpm,
                    "steer_angle": current_steer_angle
                }
                pub_socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
                pub_socket.send_json(command_out)
                mode_str = "AUTO" if is_auto_mode else "MANUAL"
                if is_auto_mode:
                    real_steer_angle = auto_steer_angle / 4
                    print(f"\rReal steer angle: {real_steer_angle:<3.1f}", end="")
                print(f"\rMode: {mode_str:<6} | CMD > Speed: {current_speed_rpm:<5.1f} RPM, Steer: {current_steer_angle:<3.0f} deg", end="")

            # time.sleep(0.02)

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # Send a final stop command
        stop_command = {"speed_rpm": 0.0, "steer_angle": 0.0}
        pub_socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
        pub_socket.send_json(stop_command)
        time.sleep(0.1)
        
        pub_socket.close()
        sub_socket.close()
        context.term()
        restore_terminal_settings(settings)
        print("\nCleanly shut down teleop controller.")

if __name__ == "__main__":
    main()