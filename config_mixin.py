# config_mixin.py
import yaml
import logging
from typing import Dict, Any

class ConfigMixin:
    """Mixin class to provide unified configuration loading capabilities."""
    
    def __init__(self, config_path: str = "config.yaml"):
        self.config_path = config_path
        self.config = self._load_config()
        
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
                logging.info(f"Successfully loaded configuration from {self.config_path}")
                return config
        except FileNotFoundError:
            logging.error(f"Configuration file not found: {self.config_path}")
            raise
        except yaml.YAMLError as e:
            logging.error(f"Error parsing YAML configuration: {e}")
            raise
        except Exception as e:
            logging.error(f"Unexpected error loading configuration: {e}")
            raise
    
    def get_zmq_url(self, key: str) -> str:
        """Get ZMQ URL from configuration."""
        return self.config['zmq'][key]
    
    def get_zmq_topic(self, key: str) -> str:
        """Get ZMQ topic from configuration."""
        return self.config['zmq'][key]
    
    def get_camera_config(self, camera_type: str = 'forward') -> Dict[str, Any]:
        """Get camera configuration."""
        return self.config['camera'][camera_type]
    
    def get_section_config(self, section: str) -> Dict[str, Any]:
        """Get configuration for a specific section."""
        return self.config.get(section, {})