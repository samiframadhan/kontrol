import numpy as np

class StateEstimator:
    def __init__(self, initial_state: dict, wheelbase: float = 1.0):
        # x, y, theta, speed, steer_angle
        if not isinstance(initial_state, dict):
            raise ValueError("Initial state must be a dictionary.")
        if not all(key in initial_state for key in ['x', 'y', 'theta', 'vx']):
            raise ValueError("Initial state must contain keys: 'x', 'y', 'theta', 'vx'.")
        self.state = initial_state
        self.wheelbase = wheelbase

    def update(self, measurement):
        # Placeholder for state update logic
        self.state['x'] = self.state['x'] + measurement.get('vx', 0.0) * measurement.get('dt', 1.0) * np.cos(self.state['theta'])
        self.state['y'] = self.state['y'] + measurement.get('vx', 0.0) * measurement.get('dt', 1.0) * np.sin(self.state['theta'])
        self.state['theta'] = self.state['theta'] + (measurement.get('vx',0.0) / self.wheelbase) * np.tan(measurement.get('steer_angle', 0.0)) * measurement.get('dt', 1.0)
        self.state['vx'] = measurement.get('vx', self.state['vx'])
        return self.state

    def get_state(self):
        return self.state
    
