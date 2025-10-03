import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from llc_interface import LLCInterfaceNode

if __name__ == "__main__":
    llc_node = LLCInterfaceNode(config_path="../params/config.yaml")
    llc_node.run()