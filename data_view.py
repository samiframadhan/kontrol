# data_viewer.py
import zmq
import json
import time

# --- ZMQ Configuration ---
ZMQ_SUB_URL = "tcp://localhost:5556"

def main():
    """Subscribes to and displays sensor data."""
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_SUB_URL)
    socket.setsockopt_string(zmq.SUBSCRIBE, "sensor_data") # Subscribe to the 'sensor_data' topic

    print(f"Subscribed to sensor data on {ZMQ_SUB_URL}\n")
    
    try:
        while True:
            # Read topic and JSON payload
            topic, data_json = socket.recv_multipart()
            data = json.loads(data_json)
            
            # --- Pretty print the received dictionary ---
            status_line = (
                f"RPM: {data.get('rpm', 'N/A'):<5.1f} | "
                f"Steer: {data.get('steer_angle', 'N/A'):<3} | "
                f"Battery: {data.get('battery_%', 'N/A')}% | "
                f"Brake: {data.get('brake_%', 'N/A')} | "
                f"US Front: {data.get('ultrasonic_front_m', 'N/A'):.2f}m"
            )
            print(f"\rRECV > {status_line}", end="")
            
    except KeyboardInterrupt:
        print("\n\nViewer stopped.")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()