import zmq
import time
import threading
import json
import logging

from managednode import ManagedNode, POLL_TIMEOUT

# --- Configuration ---
ZMQ_STEER_SUB_URL = "tcp://localhost:5558"  # URL for linefollowing_rs steering output
ZMQ_ARUCO_SUB_URL = "tcp://localhost:5555"  # URL for aruco.py marker output
ZMQ_LLC_PUB_URL = "ipc:///tmp/llc.ipc"      # URL to publish commands to llc_interface.py

# --- Driving Parameters ---
MAX_SPEED_RPM = 50
MIN_DISTANCE_STOP = 0.50  # 50 cm
BRAKE_DISTANCE = 0.10     # 10 cm
MAX_BRAKE_FORCE = 50
IGNORE_DURATION = 10.0    # seconds
CONTROL_LOOP_FREQUENCY = 0.1 # seconds

class AutonomousDrivingNode(ManagedNode):
    """
    Manages autonomous start/stop behavior based on ArUco markers and user input,
    and is controlled by a central orchestrator.
    """
    def __init__(self):
        super().__init__(node_name='autonomous') # Call parent constructor
        
        # --- ZMQ objects - initialized in on_configure ---
        self.steer_sub = None
        self.aruco_sub = None
        self.llc_pub = None
        self.subscriber_poller = zmq.Poller()
        
        # --- State Management ---
        self.current_steering_angle = 0.0
        self.current_speed_rpm = MAX_SPEED_RPM
        self.current_brake = 0
        
        self.is_stopped = False
        self.stopping_marker_id = None
        self.ignored_markers = {}  # {marker_id: timestamp}

        # --- Threading ---
        self.processing_threads = []

    # --- ManagedNode Implementation ---

    def on_configure(self) -> bool:
        """
        Load configuration, initialize ZMQ sockets and other resources.
        """
        self.logger.info("Configuring autonomous driving node...")
        try:
            # Subscriber for steering angle
            self.steer_sub = self.context.socket(zmq.SUB)
            self.steer_sub.connect(ZMQ_STEER_SUB_URL)
            self.steer_sub.setsockopt_string(zmq.SUBSCRIBE, "steering")

            # Subscriber for ArUco marker data
            self.aruco_sub = self.context.socket(zmq.SUB)
            self.aruco_sub.connect(ZMQ_ARUCO_SUB_URL)
            self.aruco_sub.setsockopt_string(zmq.SUBSCRIBE, "marker_data")

            # Publisher for LLC commands
            self.llc_pub = self.context.socket(zmq.PUB)
            self.llc_pub.connect(ZMQ_LLC_PUB_URL)

            # Register sockets with poller
            self.subscriber_poller.register(self.steer_sub, zmq.POLLIN)
            self.subscriber_poller.register(self.aruco_sub, zmq.POLLIN)
            
            self.logger.info("ZMQ sockets configured successfully.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to configure ZMQ sockets: {e}")
            return False

    def on_activate(self) -> bool:
        """
        Start the main processing loops (subscribers, control, user input).
        """
        self.logger.info("Activating autonomous driving node...")
        # Reset state on activation
        self.is_stopped = False
        self.current_brake = 0
        self.current_speed_rpm = MAX_SPEED_RPM

        # Start background threads
        subscriber_thread = threading.Thread(target=self._subscriber_loop, daemon=True)
        control_thread = threading.Thread(target=self._control_loop, daemon=True)
        user_input_thread = threading.Thread(target=self._user_input_listener, daemon=True)
        
        self.processing_threads = [subscriber_thread, control_thread, user_input_thread]
        
        for thread in self.processing_threads:
            thread.start()
            
        self.logger.info("All processing threads started.")
        return True

    def on_deactivate(self) -> bool:
        """
        Pause the main processing loops and bring the vehicle to a safe stop.
        """
        self.logger.info("Deactivating autonomous driving node...")
        # The state change to 'inactive' will cause the loops to exit.
        # Give threads a moment to exit their loops gracefully.
        time.sleep(POLL_TIMEOUT / 1000 * 2)

        # Bring vehicle to a full stop
        self.logger.info("Stopping vehicle...")
        self.current_speed_rpm = 0
        self.current_brake = 100
        self.send_llc_command()
        
        for thread in self.processing_threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        self.processing_threads = []
        self.logger.info("Processing threads stopped.")
        return True

    def on_shutdown(self) -> bool:
        """
        Release all resources.
        """
        self.logger.info("Shutting down autonomous driving node...")
        # Ensure deactivation is complete before shutting down
        if self.state == 'active':
            self.on_deactivate()

        try:
            self.steer_sub.close()
            self.aruco_sub.close()
            self.llc_pub.close()
            self.logger.info("ZMQ sockets closed.")
        except Exception as e:
            self.logger.error(f"Error closing sockets: {e}")
        return True

    # --- Core Logic Methods ---
    
    def _subscriber_loop(self):
        """
        Listens for ZMQ messages from steering and ArUco nodes.
        Runs while the node state is 'active'.
        """
        self.logger.info("Subscriber loop started.")
        while self.state == 'active':
            try:
                socks = dict(self.subscriber_poller.poll(timeout=POLL_TIMEOUT))
            except zmq.ZMQError:
                break # Context was terminated

            if self.steer_sub in socks:
                topic, msg = self.steer_sub.recv_multipart()
                self.current_steering_angle = float(msg.decode('utf-8'))

            if self.aruco_sub in socks:
                topic, msg = self.aruco_sub.recv_multipart()
                marker_data = json.loads(msg.decode('utf-8'))
                self._handle_aruco_marker(marker_data)
        self.logger.info("Subscriber loop terminated.")
        
    def _control_loop(self):
        """
        Main control loop for driving logic.
        Runs while the node state is 'active'.
        """
        self.logger.info("Control loop started.")
        while self.state == 'active':
            if not self.is_stopped:
                self.current_speed_rpm = MAX_SPEED_RPM
                self.current_brake = 0

                # Safety feature: Reduce speed based on steering angle
                steer_reduction_factor = 0.5 
                speed_reduction = abs(self.current_steering_angle) * steer_reduction_factor
                effective_speed = self.current_speed_rpm - speed_reduction
                self.current_speed_rpm = max(0, effective_speed)

            self.send_llc_command()
            
            self.logger.debug(
                f"STATE | Speed: {self.current_speed_rpm:.1f} RPM, "
                f"Steer: {self.current_steering_angle:.1f} deg, "
                f"Brake: {self.current_brake}%, "
                f"Stopped: {self.is_stopped}"
            )
            time.sleep(CONTROL_LOOP_FREQUENCY)
        self.logger.info("Control loop terminated.")

    def _user_input_listener(self):
        """
        Simple keyboard listener to resume driving.
        Runs while the node state is 'active'.
        """
        self.logger.info("User input listener started. Press 'c' and Enter to continue driving.")
        while self.state == 'active':
            try:
                # This is a blocking call, which is not ideal.
                # In a real application, a non-blocking input method would be better.
                # For this example, we check the state after the input call.
                user_input = input() 
                if self.state != 'active':
                    break
                if user_input.strip().lower() == 'c':
                    self.resume_driving()
            except (EOFError, KeyboardInterrupt):
                break
        self.logger.info("User input listener terminated.")

    def _handle_aruco_marker(self, data):
        """
        Logic to decide whether to stop based on ArUco marker data.
        """
        marker_id = data['id']
        distance = data['position'][2]

        if marker_id in self.ignored_markers:
            if time.time() - self.ignored_markers[marker_id] > IGNORE_DURATION:
                del self.ignored_markers[marker_id]
            else:
                return

        if not self.is_stopped and distance < MIN_DISTANCE_STOP:
            self.is_stopped = True
            self.stopping_marker_id = marker_id
            self.logger.info(f"Detected ArUco marker {marker_id} at {distance:.2f}m. Initiating stop.")
            
        if self.is_stopped:
            # Gradual Stop Logic
            if distance < BRAKE_DISTANCE:
                self.current_speed_rpm = 0
                self.current_brake = int(((BRAKE_DISTANCE - distance) / BRAKE_DISTANCE) * MAX_BRAKE_FORCE)
                self.current_brake = min(self.current_brake, MAX_BRAKE_FORCE)
            elif distance < MIN_DISTANCE_STOP:
                self.current_brake = 0
                self.current_speed_rpm = int(((distance - BRAKE_DISTANCE) / (MIN_DISTANCE_STOP - BRAKE_DISTANCE)) * MAX_SPEED_RPM)
            else: # If we are in 'is_stopped' but marker moved out of range
                 self.current_speed_rpm = 0
                 self.current_brake = MAX_BRAKE_FORCE


    def send_llc_command(self):
        """
        Publishes the current speed, steer, and brake commands to the LLC.
        """
        command = {
            "speed_rpm": self.current_speed_rpm,
            "steer_angle": self.current_steering_angle,
            "brake_force": self.current_brake 
        }
        if self.llc_pub and not self.llc_pub.closed:
            self.llc_pub.send_string("teleop_cmd", flags=zmq.SNDMORE)
            self.llc_pub.send_json(command)

    def resume_driving(self):
        """
        Handles user input to continue the journey.
        """
        if self.is_stopped:
            self.logger.info(f"User input received. Resuming journey and ignoring marker {self.stopping_marker_id} for {IGNORE_DURATION}s.")
            self.ignored_markers[self.stopping_marker_id] = time.time()
            self.is_stopped = False
            self.current_brake = 0
            self.current_speed_rpm = MAX_SPEED_RPM # Resume at max speed
        else:
            self.logger.warning("Cannot resume, vehicle is not in a stopped state.")


if __name__ == "__main__":
    # This will setup logging from the ManagedNode
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    node = AutonomousDrivingNode()
    node.run() # This is the run() method from ManagedNode