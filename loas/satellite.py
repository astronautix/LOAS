import numpy as np
import loas
from threading import Thread
import time

class Satellite(Thread):

    def __init__(
        self,
        mesh,
        dt = 1/30,
        M0 = np.array([[0.],[0.],[0.]]),
        dw0 = np.array([[0.],[0.],[0.]]),
        J0 = 1,
        B0 = np.array([[0.],[0.],[0.]]),
        I0 = np.array([[1.],[1.],[1.]]),
        L0 = np.array([[0.],[0.],[0.]]),
        Q0 = loas.Quaternion(1,0,0,0)
    ):
        Thread.__init__(self)
        self.t = 0 #temps écoulé
        self.dt = dt
        self.Q = Q0 #quaternion de rotation de Rv par rapport à Rr
        self.L = L0 #Moment cinétique du satellite dans Rr
        self.M = M0 # moment magnétique des MC exprimé dans # REVIEW:
        self.dw = dw0 #vecteur accélération angulaire des RI exprimé dans Rv
        self.J = J0 #Moment d'inertie des RI
        self.B = B0 #Vecteur champ magnétique environnant, exprimé dans Rr
        self.I = I0 #Tenseur d'inertie du satellite exprimé dans Rv
        self.running = False,
        self.mesh = mesh

    def dQ(self,W): #renvoie la dérivée du quaternion
        qw,qx,qy,qz = self.Q[0],self.Q[1],self.Q[2],self.Q[3]
        expQ = np.array([[-qx, -qy, -qz],
                         [ qw,  qz, -qy],
                         [-qz,  qw,  qx],
                         [ qy, -qx,  qw]])
        return 1/2*np.dot(expQ,W)

    def dL(self): #renvoie la dérivée du moment cinétique avec le th du moment cinétique
        C_Rv = np.cross(self.M, self.Q.R2V(self.B), axisa=0, axisb=0,axisc=0) - self.J*self.dw #couple dans Rv du au MC et aux RW
        C_Rr = self.Q.V2R(C_Rv)
        return C_Rr

    def getNextIteration(self):
        self.L += self.dL()*self.dt #calcul du nouveau moment cinétique
        W = self.Q.V2R(np.dot(np.linalg.inv(self.I),self.Q.R2V(self.L))) #Vecteur rotation du satellite dans Rr
        Qnump = self.Q.vec() + self.dQ(W)*self.dt #calcul de la nouvelle orientation
        Qnump /= np.linalg.norm(Qnump)
        self.Q = loas.Quaternion(*Qnump[:,0])
        self.t += self.dt
        return self.Q

    def setM(self, M):
        self.M = M

    def setDW(self, dw):
        self.dw = dw

    def setB(self, B):
        self.B = B

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            self.getNextIteration()
            time.sleep(self.dt)
