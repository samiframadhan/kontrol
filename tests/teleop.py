# teleop_node.py (Updated for unified config)
import sys
import time
import json
import zmq
import threading
import steering_command_pb2
from config_mixin import ConfigMixin

# Platform-specific imports for keyboard
try:
    import tty
    import termios
    import select
    UNIX = True
except ImportError:
    import msvcrt
    UNIX = False

class TeleopNode(ConfigMixin):
    """Teleoperation node with unified configuration."""
    
    def __init__(self, config_path="config.yaml"):
        ConfigMixin.__init__(self, config_path)
        
        # Get vehicle parameters from config
        vehicle_config = self.get_section_config('vehicle_params')
        
        self.MAX_SPEED_RPM = vehicle_config.get('max_speed_rpm', 1500.0)
        self.MAX_STEER_ANGLE = 120.0
        self.MAX_BRAKE_FORCE = vehicle_config.get('max_brake_force', 100.0)
        self.SPEED_STEP = 100.0
        self.STEER_STEP = 5.0
        self.BRAKE_STEP = 2.0
        self.SEND_PERIOD = 0.02  # 50 Hz send rate
        
        # ZMQ setup
        self.context = zmq.Context()
        self.pub_socket = None
        self.sub_socket = None
        
        # State dictionary for shared state
        self.state = {
            "current_speed_rpm": 0.0,
            "current_steer_angle": 0.0,
            "brake_force": 0.0,
            "auto_steer_angle": 0.0,
            "is_auto_mode": False,
            "last_timestamp": time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        }
        
        # Threading
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.sender_thread = None

    def setup_zmq(self):
        """Setup ZMQ connections using unified config."""
        # Publisher for control commands 
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(self.get_zmq_url('control_cmd_url'))
        
        # Subscriber for lane assist commands
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(self.get_zmq_url('steering_cmd_url'))
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('steering_cmd_topic'))
        
        print(f"Publishing control commands on {self.get_zmq_url('control_cmd_url')}")
        print(f"Subscribing to lane assist from {self.get_zmq_url('steering_cmd_url')}")

    def periodic_sender(self):
        """Runs in a separate thread to periodically send commands."""
        while not self.stop_event.is_set():
            with self.state_lock:
                # Safely copy the current state for sending
                speed_to_send = self.state["current_speed_rpm"]
                steer_to_send = self.state["current_steer_angle"]
                brake_to_send = self.state["brake_force"]
                is_auto = self.state["is_auto_mode"]
                auto_angle = self.state["auto_steer_angle"]

            command_out = {
                "speed_rpm": speed_to_send,
                "steer_angle": steer_to_send,
                "brake_force": brake_to_send
            }
            self.pub_socket.send_string(self.get_zmq_topic('control_cmd_topic'), flags=zmq.SNDMORE)
            self.pub_socket.send_json(command_out)
            
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
            
            print(display_str, end=" " * 5)  # Whitespace to clear previous line

            time.sleep(self.SEND_PERIOD)

    def getKey(self, settings, timeout=0.01):
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
            time.sleep(timeout)
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8')
            else:
                key = ''
        return key

    def save_terminal_settings(self):
        """Saves the current terminal settings."""
        if UNIX:
            return termios.tcgetattr(sys.stdin)
        return None

    def restore_terminal_settings(self, old_settings):
        """Restores terminal settings to their original state."""
        if UNIX:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def run(self):
        """Main teleop loop."""
        settings = self.save_terminal_settings()
        
        try:
            self.setup_zmq()
            
            # Start the periodic sender thread
            self.sender_thread = threading.Thread(
                target=self.periodic_sender,
                name="PeriodicSenderThread"
            )
            self.sender_thread.start()
            
            print("\nControls:")
            print("  w/s:   Increase/decrease speed")
            print("  a/d:   Steer left/right (in MANUAL mode)")
            print("  m:     Toggle between MANUAL and AUTO mode")
            print("  space: Emergency stop (set speed to 0, engage gradual brake)")
            print("  x:     Reset speed, steering, and brake to 0")
            print("  q:     Quit")
            print("-------------------------------------------")

            while True:
                # Check for incoming autonomous commands (non-blocking)
                try:
                    topic, serialized_data = self.sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                    command = steering_command_pb2.SteeringCommand()
                    command.ParseFromString(serialized_data)
                    received_angle = command.auto_steer_angle
                    
                    with self.state_lock:
                        self.state["auto_steer_angle"] = received_angle * -4.0

                except zmq.Again:
                    pass  # No message received, just continue

                # Get keyboard input
                key = self.getKey(settings)
                if key == 'q':
                    print("\nExiting program...")
                    break

                # Update state based on input
                with self.state_lock:
                    if key == 'm':
                        self.state["is_auto_mode"] = not self.state["is_auto_mode"]
                        mode_str = "AUTO" if self.state["is_auto_mode"] else "MANUAL"
                        print(f"\n--- Switched to {mode_str} mode ---")

                    # If user intends to move, disengage the brake
                    if key in ['w', 's']:
                        self.state["brake_force"] = 0.0
                    
                    # Handle all other key presses
                    if key == 'w':
                        self.state["current_speed_rpm"] = min(
                            self.state["current_speed_rpm"] + self.SPEED_STEP, 
                            self.MAX_SPEED_RPM
                        )
                    elif key == 's':
                        self.state["current_speed_rpm"] = max(
                            self.state["current_speed_rpm"] - self.SPEED_STEP, 
                            -self.MAX_SPEED_RPM
                        )
                    elif key == 'a' and not self.state["is_auto_mode"]:
                        self.state["current_steer_angle"] = max(
                            self.state["current_steer_angle"] - self.STEER_STEP, 
                            -self.MAX_STEER_ANGLE
                        )
                    elif key == 'd' and not self.state["is_auto_mode"]:
                        self.state["current_steer_angle"] = min(
                            self.state["current_steer_angle"] + self.STEER_STEP, 
                            self.MAX_STEER_ANGLE
                        )
                    elif key == ' ':
                        self.state["current_speed_rpm"] = 0.0  # Setting speed to 0 will trigger the brake logic
                    elif key == 'x':
                        self.state["current_speed_rpm"] = 0.0
                        self.state["current_steer_angle"] = 0.0
                        self.state["brake_force"] = 0.0  # Explicitly reset brake to 0

                    # Gradual Brake Application Logic
                    if self.state["current_speed_rpm"] == 0.0:
                        # If speed is zero, gradually increase brake force up to the max
                        if self.state["brake_force"] < self.MAX_BRAKE_FORCE:
                            self.state["brake_force"] = min(
                                self.state["brake_force"] + self.BRAKE_STEP, 
                                self.MAX_BRAKE_FORCE
                            )
                    
                    # In auto mode, the steering angle smoothly follows the auto angle
                    if self.state["is_auto_mode"]:
                        self.state["current_steer_angle"] = self.state["auto_steer_angle"]
                
        except (KeyboardInterrupt, SystemExit):
            print("\nCtrl+C detected. Exiting...")
        except Exception as e:
            print(f"\nAn unexpected error occurred: {e}")
        finally:
            # Graceful Shutdown
            print("\nInitiating shutdown...")
            self.stop_event.set()  # Signal the sender thread to stop
            if self.sender_thread:
                self.sender_thread.join()  # Wait for the sender thread to finish

            # Send a final stop command with full brakes
            stop_command = {"speed_rpm": 0.0, "steer_angle": 0.0, "brake_force": 100.0}
            self.pub_socket.send_string(self.get_zmq_topic('control_cmd_topic'), flags=zmq.SNDMORE)
            self.pub_socket.send_json(stop_command)
            time.sleep(0.1)  # Allow time for the message to be sent
            
            self.pub_socket.close()
            self.sub_socket.close()
            self.context.term()
            self.restore_terminal_settings(settings)
            print("Cleanly shut down teleop controller.")

if __name__ == "__main__":
    teleop = TeleopNode()
    teleop.run()