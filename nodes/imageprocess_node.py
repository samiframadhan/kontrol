import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from linefollowing import LineFollowingNode

if __name__ == "__main__":
    line_node = LineFollowingNode(config_path="../params/config.yaml")
    line_node.run()