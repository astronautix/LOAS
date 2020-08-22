import numpy as np
class Vec(np.ndarray):
    def __new__(cls,*args):
        return np.asarray(np.array([[i] for i in args], dtype='float64')).view(cls)

    def line(self):
        return list(self[:,0])

    def cross(self,b):
        return np.cross(self, b, axisa=0, axisb=0,axisc=0)

    def normalized(self):
        return self/np.linalg.norm(self)
