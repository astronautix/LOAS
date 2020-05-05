import numpy as np

def tov(*args):
    return np.array([[i] for i in args], dtype='float64')

def tol(elt):
    return elt[:,0]

def cross(a,b):
    return np.cross(a, b, axisa=0, axisb=0,axisc=0)
