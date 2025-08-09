import zmq
import time
import logging
import threading
import msgpack
from managednode import ManagedNode
from config_mixin import ConfigMixin
from shared_enums import NodeState # Assuming shared_enums.py exists

class LidarProcessorNode(ManagedNode, ConfigMixin):
    """
    A managed node that processes raw LIDAR data and publishes actionable commands.
    """
    def __init__(self, node_name="lidar_processor_node", config_path="config.yaml"):
        # Initialize parent classes
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        # ZMQ Sockets (will be created in on_configure)
        self.lidar_raw_sub = None
        self.direction_sub = None
        self.lidar_cmd_pub = None
        self.processor_poller = zmq.Poller()

        # Processing thread management
        self.processing_thread = None
        self.active_event = threading.Event()

        # State variables
        self.is_reverse = False
        self.last_direction_update = time.time()
        
        # Load parameters from config
        self.direction_timeout = self.get_section_config('lidar_processor')['direction_timeout']
        lidar_params = self.get_section_config('lidar_control')
        self.min_dist_stop = lidar_params['min_distance_stop']
        self.min_dist_reduce = lidar_params['min_distance_reduce_speed']
        self.points_percentage = lidar_params['points_percentage_for_average']

    def on_configure(self) -> bool:
        self.logger.info("Configuring Lidar Processor Node...")
        try:
            # 1. Subscribe to raw LIDAR data from simple.rs
            self.lidar_raw_sub = self.context.socket(zmq.SUB)
            self.lidar_raw_sub.connect(self.get_zmq_url('lidar_raw_url'))
            self.lidar_raw_sub.setsockopt_string(zmq.SUBSCRIBE, "") # All topics
            self.processor_poller.register(self.lidar_raw_sub, zmq.POLLIN)

            # 2. Subscribe to direction updates from control_node
            self.direction_sub = self.context.socket(zmq.SUB)
            self.direction_sub.connect(self.get_zmq_url('direction_status_url'))
            self.direction_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('direction_status_topic'))
            self.processor_poller.register(self.direction_sub, zmq.POLLIN)

            # 3. Publish processed commands to control_node
            self.lidar_cmd_pub = self.context.socket(zmq.PUB)
            self.lidar_cmd_pub.bind(self.get_zmq_url('lidar_command_url'))
            
            self.logger.info("Sockets configured successfully.")
            return True
        except Exception as e:
            self.logger.error(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating Lidar Processor Node...")
        self.active_event.set()
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating Lidar Processor Node...")
        self.active_event.clear()
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
        self.logger.info("Node deactivated.")
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Lidar Processor Node...")
        if self.state == NodeState.ACTIVE:
            self.on_deactivate()
        
        # Close all sockets
        if self.lidar_raw_sub: self.lidar_raw_sub.close()
        if self.direction_sub: self.direction_sub.close()
        if self.lidar_cmd_pub: self.lidar_cmd_pub.close()
        
        self.logger.info("Shutdown complete.")
        return True

    def _processing_loop(self):
        """The main loop for polling sockets and processing data."""
        self.logger.info("Lidar processing loop started.")
        while self.active_event.is_set():
            socks = dict(self.processor_poller.poll(100))

            # Handle direction updates first
            if self.direction_sub in socks:
                topic, msg = self.direction_sub.recv_multipart()
                self.is_reverse = msg.decode('utf-8') == "REVERSE"
                self.last_direction_update = time.time()
                self.logger.debug(f"Direction updated: {'REVERSE' if self.is_reverse else 'FORWARD'}")

            # Check for direction data timeout
            if time.time() - self.last_direction_update > self.direction_timeout:
                self.logger.warning("Direction data timed out. Assuming FORWARD for safety.")
                self.is_reverse = False

            # Handle LIDAR data
            if self.lidar_raw_sub in socks:
                topic_bytes, packed_data = self.lidar_raw_sub.recv_multipart()
                command = self._calculate_command(topic_bytes, packed_data)
                
                # Publish the command if one was generated
                if command:
                    self.lidar_cmd_pub.send_string(self.get_zmq_topic('lidar_command_topic'), flags=zmq.SNDMORE)
                    self.lidar_cmd_pub.send_string(command)
        
        self.logger.info("Lidar processing loop stopped.")

    def _calculate_command(self, topic_bytes, packed_data):
        """Processes a single LIDAR scan and returns a command string."""
        topic = topic_bytes.decode('utf-8')
        is_primary = (not self.is_reverse and "depan" in topic) or \
                     (self.is_reverse and "belakang" in topic)

        if not is_primary:
            return None # No command for non-primary LIDAR
        
        # Determine the source for the command string
        source = "DEPAN" if "depan" in topic else "BELAKANG"

        try:
            points = msgpack.unpackb(packed_data)
            if not points: return "GO"

            sorted_points = sorted(points, key=lambda p: p[2])
            num_to_avg = max(1, int(len(sorted_points) * self.points_percentage))
            avg_dist = sum(p[2] for p in sorted_points[:num_to_avg]) / num_to_avg

            self.logger.debug(f"Primary LIDAR ({topic}) avg dist: {avg_dist:.2f}m")

            if avg_dist < self.min_dist_stop:
                self.logger.warning(f"Obstacle too close ({avg_dist:.2f}m). Commanding STOP from {source}.")
                return f"STOP_{source}"
            elif avg_dist < self.min_dist_reduce:
                self.logger.info(f"Obstacle in range ({avg_dist:.2f}m). Commanding REDUCE_SPEED from {source}.")
                return f"REDUCE_SPEED_{source}"
            else:
                return "GO"
        except Exception as e:
            self.logger.error(f"Error unpacking/processing LIDAR data: {e}")
            return f"STOP_{source}" # Default to STOP on error for safety

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    node = LidarProcessorNode()
    node.run()