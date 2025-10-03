import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from hmi import HMINode

if __name__ == "__main__":
    hmi_node = HMINode(config_path="../params/config.yaml")
    hmi_node.run()