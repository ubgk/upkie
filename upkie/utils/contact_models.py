import numpy as np
from scipy.interpolate import interpn


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
        return interpn(points=[self.wheel_torques, self.knee_torques], values=self.contact_probabilities, xi=xi)[0]

    def cycle(self, observation: dict, dt: float) -> float:
        """
        Update the measurement model with a new observation.

        Args:
            observation: Latest observation.
            dt: Duration in seconds until next cycle.

        Returns:
            Contact probability.
        """
        self._left_wheel_torque = observation['joint_filter']['left_wheel']['torque'] #* 0
        self._left_knee_torque = observation['joint_filter']['left_knee']['torque'] #* 0

        try:
            self.p_contact = self.get_contact_probability([self._left_wheel_torque, self._left_knee_torque])
        except:
            raise ValueError(f"Error in contact model. Inputs are {self._left_wheel_torque} and {self._left_knee_torque}!")
            

        return self.p_contact
    
    def log(self) -> dict:
        return {
            "wheel_torque": self._left_wheel_torque,
            "knee_torque": self._left_knee_torque,
            "contact_probability": self.p_contact,
        }

