import numpy as np
import loas
from threading import Thread
import time

class Satellite(Thread):

    """
    The satellite class, represents the simulated satellite itself.

    Stores the shape of the satellite, its mass properties, current attitude and angular velocity.

    Can be used directly as loas.Satellite
    """

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
        Q0 = loas.utils.Quaternion(1,0,0,0),
        output = None
    ):
        """
        :param mesh: Mesh of the satellite
        :type mesh: trimesh.Trimesh
        :param dt: Time step of the simulation
        :type dt: float
        :param M0: Initial magnetic moment of the magnetorquer
        :type M0: (3,1) numpy array
        :param dw0: Initial rotation speed vector of the reaction wheels
        :type dw0: (3,1) numpy array
        :param J0: Inertia momentum of reaction wheel
        :type J0: float
        :param B0: Initial external magnetic field
        :type M0: (3,1) numpy array
        :param I0: Initial inertia matrix of the satellite
        :type I0: (3,1) numpy array
        :param L0: Initial angular momentum of the satellite
        :type L0: (3,1) numpy array
        :param Q0: Initial attitude of the satellite
        :type Q0: (3,1) numpy array
        :param output: loas.output.Output inheriting class' instance (e.g. Viewer). Defines how to output the simu
        :type callback: loas.output.Output inherited class' instance
        """
        Thread.__init__(self)
        self.t = 0 #temps écoulé
        self.dt = dt
        self.Q = Q0 #quaternion de rotation de Rv par rapport à Rr
        self.L = L0 #Moment cinétique du satellite dans Rr
        self.M = M0 # moment magnétique des MC exprimé dans Rv
        self.dw = dw0 #vecteur accélération angulaire des RI exprimé dans Rv
        self.J = J0 #Moment d'inertie des RI
        self.B = B0 #Vecteur champ magnétique environnant, exprimé dans Rr
        self.I = I0 #Tenseur d'inertie du satellite exprimé dans Rv
        self.running = False
        self.mesh = mesh
        self.parasite_torques = []
        self.output = output

    @property
    def W(self):
        return self.Q.V2R(np.linalg.inv(self.I) @ self.Q.R2V(self.L))

    def _dL(self): #renvoie la dérivée du moment cinétique avec le th du moment cinétique
        """
        Gives the first derivative of the angular momentum (uses dynamic equations)
        """
        C_Rv_no_parasite = np.cross(self.M, self.Q.R2V(self.B), axisa=0, axisb=0,axisc=0) - self.J*self.dw #couple dans Rv du au MC et aux RW
        C_Rr = self.Q.V2R(C_Rv_no_parasite) + sum([torque.getTorque() for torque in self.parasite_torques])
        return C_Rr

    def iterate(self):
        """
        Update the instance at the next simulation iteration
        """
        self.L += self._dL()*self.dt #calcul du nouveau moment cinétique
        Qnump = self.Q.vec() + self.Q.derivative(self.W)*self.dt #calcul de la nouvelle orientation
        Qnump /= np.linalg.norm(Qnump)
        self.Q = loas.utils.Quaternion(*Qnump[:,0])
        self.t += self.dt

        if self.output is not None:
            self.output.update(
                t = self.t,
                satellite = self
            )

    def addParasiteTorque(self,parasite_torque):
        self.parasite_torques.append(parasite_torque)

    def stop(self):
        """
        Breaks the thread
        """
        self.running = False

    def run(self):
        """
        Launches the simulation in a thread
        """
        self.running = True

        frames = 0
        time_frames = 0

        while self.running:
            t1 = time.time()

            self.iterate()

            t2 = time.time()

            deltat = t2 - t1
            pause = max(0, self.dt - deltat)

            time.sleep(pause)

            frames += 1
            time_frames += deltat + pause
            if time_frames > 1:
                print(round(frames/time_frames, 3), "/", round(1/self.dt), "FPS    ", end="\r")
                frames = 0
                time_frames = 0
