import numpy as np
from scipy.interpolate import interpn
from scipy import signal

import gin

@gin.configurable
class MeasurementModel:
    """
    Measurement model for Upkie's leg torques.
    """
    def __init__(self, model_path: str) -> None:
        """
        Load the model from a file.

        Args:
            model_path: Path to the precomputed measurement model.
        """
        data = np.load(model_path)
        self.wheel_torques = data['wheel_torques']
        self.knee_torques = data['knee_torques']
        self.contact_probabilities = data['P_contact']

    def get_contact_probability(self, xi: list[float]) -> float:
        """
        Lookup the contact probability for a given wheel and knee torque.

        Args:
            xi: Wheel and knee torque.

        Returns:
            Contact probability.
        """
        return interpn(points=[self.wheel_torques, self.knee_torques], 
                       values=self.contact_probabilities, xi=xi, bounds_error=False)[0]

    def cycle(self, observation: dict) -> float:
        """
        Update the measurement model with a new observation.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Contact probability.
        """
        self._left_wheel_torque = observation['joint_filter']['left_wheel']['torque']
        self._left_knee_torque = observation['joint_filter']['left_knee']['torque']

        self.p_contact = self.get_contact_probability([self._left_wheel_torque, self._left_knee_torque])

        return self.p_contact
    
    def log(self) -> dict:
        return {
            "wheel_torque": self._left_wheel_torque,
            "knee_torque": self._left_knee_torque,
            "contact_probability": self.p_contact,
        }

@gin.configurable
class TransitionModel:
    """
    Transition model for vertical acceleration measurements.
    """
    def __init__(self, 
                 transition_midpoint: float, 
                 transition_slope: float,
                 touchdown_midpoint: float, 
                 touchdown_slope: float,
                 window_size: int) -> None:
        """
        Load the model from a file.

        Args:
            model_path: Path to the precomputed measurement model.
        """
        self.transition_midpoint = transition_midpoint
        self.transition_slope = transition_slope
        self.touchdown_midpoint = touchdown_midpoint
        self.touchdown_slope = touchdown_slope

        self.window_size = window_size
        self.vertical_accelerations = np.zeros(window_size)
        self.timestamps = np.zeros(window_size)

        self.p_touchdown = -1
        self.p_transition = -1
        self.conditional_p_touchdown = -1
        self.conditional_p_takeoff = -1
        self.window_len = -1

    @staticmethod
    def logistic(x: float, midpoint: float, slope: float) -> float:
        return 1 / (1 + np.exp(-slope * (x - midpoint)))

    def cycle(self, observation: dict) -> tuple[float, float]:
        """
        Update the measurement model with a new observation.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Contact probability.
        """
        # Update the vertical acceleration buffer
        self.vertical_accelerations = np.roll(self.vertical_accelerations, -1)
        self.vertical_accelerations[-1] = observation['imu']['linear_acceleration'][2]

        # Update the timestamp buffer
        self.timestamps = np.roll(self.timestamps, -1)
        self.timestamps[-1] = observation['time']

        # Compute the spectrogram        
        self.avg_freq = 1 / np.mean(np.diff(self.timestamps))

        # Take the last one second of data in the buffer
        dt = self.timestamps[-1] - self.timestamps
        mask = dt < 1
        window = self.vertical_accelerations[mask]

        self.window_len = len(window)

        print(self.avg_freq)
        f, t, spectogram = signal.spectrogram(window, fs=self.avg_freq, scaling='density', mode='psd', nperseg=self.window_size, noverlap=0)

        # Check that we only have one window
        assert spectogram.shape[1] == 1 and t.shape == (1,), "We only expect one window at a time!"

        # Compute the total power in the window, and the probability of a transition in any direction
        self.total_power = spectogram.sum()
        self.p_transition = self.logistic(self.total_power, self.transition_midpoint, self.transition_slope)

        # Compute the median frequency
        norm_spectogram = spectogram / self.total_power
        spectogram_cdf = np.cumsum(norm_spectogram, axis=0)
        median_freq = f[np.argmin(np.abs(spectogram_cdf - 0.5))]

        # Compute the conditional probabilities of touchdown and takeoff
        self.conditional_p_touchdown = self.logistic(median_freq, self.touchdown_midpoint, self.touchdown_slope)
        self.conditional_p_takeoff = 1 - self.conditional_p_touchdown
        
        # Compute the marginal probability of touchdown and takeoff
        self.p_touchdown = self.p_transition * self.conditional_p_touchdown
        self.p_takeoff = self.p_transition * self.conditional_p_takeoff

        return self.p_touchdown, self.p_takeoff

    def log(self) -> dict:
        return {
            "p_transition": self.p_transition,
            "total_power": self.total_power,
            "conditional_p_touchdown": self.conditional_p_touchdown,
            "conditional_p_takeoff": self.conditional_p_takeoff,
            "p_touchdown": self.p_touchdown,
            "p_takeoff": self.p_takeoff,
            "window_len": self.window_len,
        }
    