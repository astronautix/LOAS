import numpy as np
from math import acos, sqrt, atan2, asin,pi
import trimesh
import copy
import shapely.ops
import shapely.geometry
import memoization

import loas

@memoization.cached
def projected_area(mesh, normal):
    normal = normal/np.linalg.norm(normal)
    m = copy.deepcopy(mesh)
    dir_rot = normal + loas.utils.tov(1,0,0)
    if np.linalg.norm(dir_rot) < 1e-6:
        dir_rot = loas.utils.tov(0,1,0)
    m.apply_transform(trimesh.transformations.rotation_matrix(pi, loas.utils.tol(dir_rot)))
    m.apply_transform(trimesh.transformations.projection_matrix((0,0,0),(1,0,0)))
    polygons = [
        shapely.geometry.Polygon(triangle[:,1:])
        for index_triangle, triangle in enumerate(m.triangles)
        if np.linalg.norm(m.face_normals[index_triangle] - np.array([1,0,0])) < 1e-6
    ]
    poly_merged = shapely.ops.unary_union(polygons)
    return poly_merged.area

def tov(*args):
    return np.array([[i] for i in args], dtype='float64')

def tol(elt):
    return elt[:,0]

def cross(a,b):
    return np.cross(a, b, axisa=0, axisb=0,axisc=0)

class Quaternion:
    """
    Allows a simple use of quaternion object to represent object's attitude

    Can be used directly as loas.Quaternion
    """
    def __init__(self,a,b,c,d):
        """
        Normalize the quaternion at initialization.

        :param a: q_0
        :type a: float
        :param b: q_1
        :type b: float
        :param c: q_2
        :type c: float
        :param d: q_3
        :type d: float
        """
        norm = sqrt(a**2+b**2+c**2+d**2)
        self.a = float(a/norm)  #: first element of the Quaternion q_0
        self.b = float(b/norm)  #: second element of the Quaternion q_1
        self.c = float(c/norm)  #: third element of the Quaternion q_2
        self.d = float(d/norm)  #: forth element of the Quaternion q_3
        self.tmsave = None
        self.tminvsave = None

    def inv(self):
        """
        Returns quaternion conjugate
        """
        return Quaternion(self.a,-self.b,-self.c,-self.d)

    def vec(self):
        """
        Returns a numpy array representation of the quaternion
        """
        return np.array([[self.a],[self.b],[self.c],[self.d]])

    def __mul__(self,value):
        """
        Multiplication operation between quaternions
        """
        return Quaternion(
            self.a*value.a - self.b*value.b - self.c*value.c - self.d*value.d,
            self.b*value.a + self.a*value.b - self.d*value.c + self.c*value.d,
            self.c*value.a + self.d*value.b + self.a*value.c - self.d*value.d,
            self.d*value.a - self.c*value.b + self.b*value.c + self.a*value.d,
        )

    def tm(self): #transfer matrix from Rr to Rv i.e. X_Rr = M * X_Rv
        """
        Returns the transfer matrix from reference frame to vehicle frame
        """
        if self.tmsave is None:
            q0,q1,q2,q3 = self.a,self.b,self.c,self.d
            self.tmsave = np.array(
                [[2*(q0**2+q1**2)-1, 2*(q1*q2-q0*q3)  , 2*(q1*q3+q0*q2)  ],
                 [2*(q1*q2+q0*q3)  , 2*(q0**2+q2**2)-1, 2*(q2*q3-q0*q1)  ],
                 [2*(q1*q3-q0*q2)  , 2*(q2*q3+q0*q1)  , 2*(q0**2+q3**2)-1]]
            )
        return self.tmsave

    def tminv(self): #transfer matrix from Rv to Rr i.e. X_Rv = M * X_Rr
        """
        Returns the transfer matrix from vehicle frame to reference frame
        """
        if self.tminvsave is None:
            self.tminvsave = np.linalg.inv(self.tm())
        return self.tminvsave

    def __getitem__(self,index):
        """
        Index getter
        """
        if index == 0:
            return self.a
        elif index == 1:
            return self.b
        elif index == 2:
            return self.c
        elif index == 3:
            return self.d
        else:
            raise IndexError("Accessing a non-existing value of a 4 elements vector")

    def derivative(self, W):
        expQ = np.array([
            [-self.b, -self.c, -self.d],
            [ self.a,  self.d, -self.c],
            [-self.d,  self.a,  self.b],
            [ self.c, -self.b,  self.a]
        ])
        return expQ @ W / 2

    def axis(self):
        """
        Return the normalized rotation axis of the quaternion
        """
        res = np.array([[self.b],[self.c],[self.d]])
        if np.linalg.norm(res) == 0:
            return np.array([[1],[0],[0]])
        return res/np.linalg.norm(res)

    def angle(self):
        """
        Returns the rotation angle of the quaternion
        """
        return acos(max(-1,min(self.a,1)))*2

    def V2R(self,vec):
        """
        Changes *vec* frame of reference from vehicle to reference

        :param vec: Input vector
        :type vec: (3,1) numpy array
        """
        return self.tm() @ vec

    def R2V(self,vec):
        """
        Changes *vec* frame of reference from reference to vehicle

        :param vec: Input vector
        :type vec: (3,1) numpy array
        """
        return self.tminv() @ vec

    def euler(self):
        """
        Returns euler angle of the rotation defined by the quaternion

        It is bad to use euler angles.
        """
        q0,q1,q2,q3 = self.a,self.b,self.c,self.d
        return np.array([[atan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2))],[asin(2*(q0*q2-q3*q1))],[atan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))]])
