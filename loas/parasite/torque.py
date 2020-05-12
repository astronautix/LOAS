from abc import ABC, abstractmethod

class Torque(ABC):
    """
    Abstract class that defines the structure of the Parasite Torques classes.
    """

    def __init__(self, satellite):
        """
        :param satellite: Satellite instance
        :type satellite: loas.Satellite
        :param viewer: Viewer instance. If set to None (or unset), nothing will be displayed.
        :type viewer: loas.output.Viewer
        """
        super().__init__()
        self.satellite = satellite

    @abstractmethod
    def getTorque(self, t):
        """
        Return the torque corresponding to the current satellite situation
        """
