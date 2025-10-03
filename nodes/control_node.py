import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from control import ControlNode

if __name__ == "__main__":
    control_node = ControlNode(config_path="../params/config.yaml")
    control_node.run()