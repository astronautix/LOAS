import sys
import os
sys.path.append(os.path.join(*['..']*2))
from math import *
import numpy as np
from simulator import Simulator
from quaternion import Quaternion
from viewer import Viewer
###############################
# Paramètres de la simulation #
###############################

from conf import *

I0 = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite

sim = Simulator(dt, dw0 = np.array([[1.],[0.],[0.]]), I0 = I0)
viewer = Viewer("teapot.obj", sim.getQ, 30)

sim.start()
viewer.run()
sim.stop()
sim.join()
