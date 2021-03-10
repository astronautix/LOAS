import numpy as np
class Vec(np.ndarray):
    """
    Represents a vector in LOAS
    Extends np.ndarray, so it can be used just as if it was a (n,1) np.array
    """

    def __new__(cls,*args):
        """
        :param *args: Elements of the vector
        :type *args: floats
        """
        return np.asarray(np.array([[i] for i in args], dtype='float64')).view(cls)

    def line(self):
        """
        A vector as a simple list
        """
        return list(self[:,0])

    def cross(self,b):
        """
        Cross operator

        :param b: The loas.Vec to perform the cross operation
        :type b: loas.Vec
        """
        return np.cross(self, b, axisa=0, axisb=0,axisc=0)

    def normalized(self):
        """
        Returns the normalized vector
        """
        return self/np.linalg.norm(self)
