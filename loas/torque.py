from abc import ABC, abstractmethod
import loas

class Torque(ABC):
    def __init__(self, satellite, viewer = None):
        super().__init__()
        self.satellite = satellite
        self.viewer = viewer

    @abstractmethod
    def getTorque(self, t):
        """
        Return the torque corresponding to the current satellite situation
        """
