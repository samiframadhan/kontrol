import zmq
import msgpack
import msgpack_numpy as m
import time
import math
import logging
import threading

from managednode import ManagedNode
from config_mixin import ConfigMixin
# Ubah nama import sesuai file proto baru (meski nama file .py nya mungkin sama)
import obstacle_data_pb2 

m.patch()

class LidarProcessorNode(ManagedNode, ConfigMixin):
    """
    Node ini mendengarkan data mentah dari sensor LIDAR via msgpack,
    mengonversi semua titik ke koordinat Kartesius, dan 
    mempublikasikan seluruh pindaian sebagai satu pesan Protobuf.
    """
    def __init__(self, node_name="lidar_processor", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.raw_lidar_sub = None
        self.scan_pub = None
        
        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        self.logger.info("Configuring LIDAR Processor Node with corrected subscriber...")
        try:
            # Setup subscriber tanpa topic filtering (menerima satu frame data msgpack saja)
            url_sub = self.get_zmq_url('lidar_raw_url')
            self.raw_lidar_sub = self.context.socket(zmq.SUB)
            self.raw_lidar_sub.connect(url_sub)
            # Subscribe ke semua messages
            self.raw_lidar_sub.setsockopt_string(zmq.SUBSCRIBE, "")
            print(f"[DEBUG] SUB socket connected to {url_sub}, no topic filter")

            # Setup publisher
            url_pub = self.get_zmq_url('obstacle_data_url')
            self.scan_pub = self.context.socket(zmq.PUB)
            self.scan_pub.bind(url_pub)
            topic_pub = self.get_zmq_topic('obstacle_data_topic')
            print(f"[DEBUG] PUB socket bound to {url_pub}, publishing on topic '{topic_pub}'")

            self.logger.info(f"Subscribed to raw LIDAR data on {url_sub} (all topics)")
            self.logger.info(f"Publishing processed LidarScan on {url_pub}")
            return True
        except Exception as e:
            self.logger.error(f"Configuration failed: {e}", exc_info=True)
            print(f"[ERROR] Configuration failed: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down LIDAR processor sockets...")
        if self.raw_lidar_sub:
            self.raw_lidar_sub.close()
            print("[DEBUG] SUB socket closed")
        if self.scan_pub:
            self.scan_pub.close()
            print("[DEBUG] PUB socket closed")
        return True

    def _processing_loop(self):
        self.logger.info("LIDAR processing loop started.")
        print("[DEBUG] Processing loop is running...")
        poller = zmq.Poller()
        poller.register(self.raw_lidar_sub, zmq.POLLIN)

        topic_pub = self.get_zmq_topic('obstacle_data_topic')
        while self.active_event.is_set():
            socks = dict(poller.poll(timeout=1000))
            if self.raw_lidar_sub in socks:
                try:
                    # Terima satu frame msgpack (tanpa topic)
                    packed_data = self.raw_lidar_sub.recv()
                    # Debug print raw bytes length
                    print(f"[DEBUG] Received raw msgpack, bytes={len(packed_data)}")

                    # Unpack menjadi list of tuples
                    raw_points = msgpack.unpackb(packed_data)
                    # print(f"[DEBUG] Unpacked {len(raw_points)} points: {raw_points[:5]}{'...' if len(raw_points)>5 else ''}")

                    # Buat pesan Protobuf LidarScan
                    lidar_scan_msg = obstacle_data_pb2.LidarScan()
                    for angle_rad, distance_m, quality in raw_points:
                        if distance_m == 0:
                            continue
                        x_m = distance_m * math.sin(angle_rad)
                        y_m = distance_m * math.cos(angle_rad)
                        pt = lidar_scan_msg.points.add()
                        pt.x_m = x_m
                        pt.y_m = y_m
                        pt.quality = int(quality)

                    # Serialize & publish
                    serialized_msg = lidar_scan_msg.SerializeToString()
                    # Kirim dua-frame: topic + payload
                    self.scan_pub.send_multipart([
                        topic_pub.encode('utf-8'),
                        serialized_msg
                    ])
                    # print(f"[DEBUG] Published processed scan on '{topic_pub}', {len(lidar_scan_msg.points)} points")

                except zmq.Again:
                    continue
                except Exception as e:
                    self.logger.error(f"Error processing LIDAR data: {e}", exc_info=True)
                    print(f"[ERROR] Processing error: {e}")

    def on_activate(self) -> bool:
        self.logger.info("Activating LIDAR processing loop...")
        print("[DEBUG] Activating processing loop")
        # Beri jeda agar publisher Rust sempat bind dan subscriber sempat subscribe
        time.sleep(0.5)
        self.active_event.set()
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating LIDAR processing loop...")
        print("[DEBUG] Deactivating processing loop")
        self.active_event.clear()
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
            print("[DEBUG] Processing thread joined")
        return True

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    print("Starting LIDAR Processor Node as a standalone process...")
    node = LidarProcessorNode()
    node.run()
