# teleop_keyboard.py
import sys
import time
import json
import zmq

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
ZMQ_PUB_URL = "tcp://*:5555"

# --- Control Settings ---
MAX_SPEED_RPM = 500.0   # The maximum speed in RPM
MAX_STEER_ANGLE = 90.0  # The maximum steering angle in degrees
SPEED_STEP = 25.0       # How much to increase/decrease speed per key press
STEER_STEP = 5.0        # How much to adjust steering per key press

def getKey(settings, timeout=0.1):
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
    """Main function to run the keyboard teleoperation."""
    settings = save_terminal_settings()

    # --- ZMQ Setup ---
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(ZMQ_PUB_URL)
    print(f"Publishing commands on {ZMQ_PUB_URL}")

    current_speed_rpm = 0.0
    current_steer_angle = 0.0

    print("\n--- Keyboard Teleoperation Control ---")
    print("       w: Increase Speed")
    print("a: Steer Left     s: Decrease Speed     d: Steer Right")
    print("       space: Brake (Set Speed to 0)")
    print("       x: Full Stop (Zero Speed & Steering)")
    print("-------------------------------------------------")
    print("Press 'q' to quit.\n")

    try:
        while True:
            key = getKey(settings)
            
            changed = False
            if key in ['w', 's', 'a', 'd', ' ', 'x']:
                changed = True
                if key == 'w':
                    current_speed_rpm = min(current_speed_rpm + SPEED_STEP, MAX_SPEED_RPM)
                elif key == 's':
                    current_speed_rpm = max(current_speed_rpm - SPEED_STEP, -MAX_SPEED_RPM)
                elif key == 'a':
                    current_steer_angle = max(current_steer_angle - STEER_STEP, -MAX_STEER_ANGLE)
                elif key == 'd':
                    current_steer_angle = min(current_steer_angle + STEER_STEP, MAX_STEER_ANGLE)
                elif key == ' ':
                    current_speed_rpm = 0.0
                elif key == 'x':
                    current_speed_rpm = 0.0
                    current_steer_angle = 0.0
            elif key == 'q':
                print("\nExiting program...")
                break
            
            # Publish commands only if they change
            if changed:
                command = {
                    "speed_rpm": current_speed_rpm,
                    "steer_angle": current_steer_angle
                }
                # Use a topic "teleop_cmd" and send the JSON string
                socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
                socket.send_json(command)

                # Display current state
                print(f"\rCMD > Speed: {current_speed_rpm:<5.1f} RPM, Steer: {current_steer_angle:<3.0f} deg", end="")

            time.sleep(0.02) # Small sleep to prevent high CPU usage

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # Send a final stop command
        stop_command = {"speed_rpm": 0.0, "steer_angle": 0.0}
        socket.send_string("teleop_cmd", flags=zmq.SNDMORE)
        socket.send_json(stop_command)
        time.sleep(0.1) # Give ZMQ time to send
        
        socket.close()
        context.term()
        restore_terminal_settings(settings)
        print("\nCleanly shut down teleop controller.")

if __name__ == "__main__":
    main()