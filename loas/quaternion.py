import numpy as np
import math
import loas

class Quat:
    """
    Allows a simple use of quaternion object to represent object's attitude

    Can be used directly as loas.Quaternion
    """
    def __init__(self,a,b,c,d):
        """
        Normalize the quaternion at initialization.
        q = q0 + i*q1 * j*q2 + k*q3

        :param a: q_0
        :type a: float
        :param b: q_1
        :type b: float
        :param c: q_2
        :type c: float
        :param d: q_3
        :type d: float
        """
        norm = math.sqrt(a**2+b**2+c**2+d**2)
        if a < 0: # from the point of view of rotations, Q is equivalent to -Q, so we normalize such as a >= 0 and thus 0 <= Q.angle <= math.pi
            norm = -norm
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
        return Quat(self.a,-self.b,-self.c,-self.d)

    def vec(self):
        """
        Returns a numpy array representation of the quaternion
        """
        return loas.Vec(self.a,self.b,self.c,self.d)

    def __mul__(self,value):
        """
        Multiplication operation between quaternions

        :param value: Another quaternion to multiply with
        :type value: loas.Quat
        """
        return Quat(
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
        """
        Get the time derivative of the quaternion with the rotation vector

        :param W: Rotation vector
        :type W: loas.Vec
        """
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
        res = loas.Vec(self.b,self.c,self.d)
        if np.linalg.norm(res) == 0:
            return loas.Vec(1,0,0)
        return res/np.linalg.norm(res)

    def angle(self):
        """
        Returns the rotation angle of the quaternion
        """
        return math.acos(max(-1,min(self.a,1)))*2

    def V2R(self,vec):
        """
        Changes *vec* frame of reference from vehicle to reference

        :param vec: Input vector
        :type vec: loas.Vec
        """
        return self.tm() @ vec

    def R2V(self,vec):
        """
        Changes *vec* frame of reference from reference to vehicle

        :param vec: Input vector
        :type vec: loas.Vec
        """
        return self.tminv() @ vec

    def euler(self):
        """
        Returns euler angle of the rotation defined by the quaternion

        It is bad to use euler angles.
        """
        q0,q1,q2,q3 = self.a,self.b,self.c,self.d
        return loas.Vec(
            math.atan2(2*(q0*q1+q2*q3),1-2*(q1**2+q2**2)),
            math.asin(2*(q0*q2-q3*q1)),
            math.atan2(2*(q0*q3+q1*q2),1-2*(q2**2+q3**2))
        )
