import sys
import os
sys.path.append(os.path.join(*['..']*2))
from math import *
import numpy as np
from simulator import Simulator
from quaternion import Quaternion
from viewer import Viewer
import trimesh
###############################
# Paramètres de la simulation #
###############################

from conf import *

sim = Simulator(dt, dw0 = np.array([[1.],[0.],[0.]]), I0 = I0)
mesh = trimesh.load_mesh("obj/bunny.stl")
mesh.apply_transform(np.eye(4)*.02) #resize mesh

viewer = Viewer( mesh, sim.getQ, 30 )

sim.start()
viewer.run()
sim.stop()
sim.join()
