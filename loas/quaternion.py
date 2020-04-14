import numpy as np
from math import acos, sqrt, atan2, asin

class Quaternion:
    """ The quaternion class

    Allows a simple use of quaternion
    """
    def __init__(self,a,b,c,d):
        """

        :param a: q_0
        :type a: float
        :param b: q_1
        :type b: float
        :param c: q_2
        :type c: float
        :param d: q_3
        :type d: float

        Normalize the quaternion at initialization.
        """
        norm = sqrt(a**2+b**2+c**2+d**2)
        self.a = a/norm  #: first element of the Quaternion q_0
        self.b = b/norm  #: second element of the Quaternion q_1
        self.c = c/norm  #: third element of the Quaternion q_2
        self.d = d/norm  #: forth element of the Quaternion q_3
        self.tmsave = None
        self.tminvsave = None

    def inv(self):
        """ Calcule le conjugué du quaternion.
            Ceux-ci étant unitaires, inverse = conjugué.
        """
        return Quaternion(self.a,-self.b,-self.c,-self.d)

    def vec(self):
        return np.array([[self.a],[self.b],[self.c],[self.d]])

    def __mul__(self,value):
        return Quaternion(
            self.a*value.a - self.b*value.b - self.c*value.c - self.d*value.d,
            self.b*value.a + self.a*value.b - self.d*value.c + self.c*value.d,
            self.c*value.a + self.d*value.b + self.a*value.c - self.d*value.d,
            self.d*value.a - self.c*value.b + self.b*value.c + self.a*value.d,
        )

    def tm(self): #transfer matrix from Rr to Rv i.e. X_Rr = M * X_Rv
        if self.tmsave is None:
            q0,q1,q2,q3 = self.a,self.b,self.c,self.d
            self.tmsave = np.array(
                [[2*(q0**2+q1**2)-1, 2*(q1*q2-q0*q3)  , 2*(q1*q3+q0*q2)  ],
                 [2*(q1*q2+q0*q3)  , 2*(q0**2+q2**2)-1, 2*(q2*q3-q0*q1)  ],
                 [2*(q1*q3-q0*q2)  , 2*(q2*q3+q0*q1)  , 2*(q0**2+q3**2)-1]]
            )
        return self.tmsave

    def tminv(self): #transfer matrix from Rv to Rr i.e. X_Rv = M * X_Rr
        if self.tminvsave is None:
            self.tminvsave = np.linalg.inv(self.tm())
        return self.tminvsave

    def __getitem__(self,index):
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

    def axis(self):
        res = np.array([[self.b],[self.c],[self.d]])
        if np.linalg.norm(res) == 0:
            return np.array([[1],[0],[0]])
        return res/np.linalg.norm(res)

    def angle(self):
        return acos(max(-1,min(self.a,1)))*2

    def V2R(self,vec):
        return np.dot(self.tm(),vec)

    def R2V(self,vec):
        return np.dot(self.tminv(),vec)

    def euler(self):
        q0,q1,q2,q3 = self.a,self.b,self.c,self.d
        return np.array([[atan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2))],[asin(2*(q0*q2-q3*q1))],[atan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))]])
