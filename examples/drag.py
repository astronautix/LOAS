import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import loas
import random
import trimesh
import threading
import time

dt = 1/25 # Simulation time step
I0 = np.diag(1,1,1) # Satellite inertia tensor

# load mesh object and resize it
mesh = trimesh.load_mesh("./satellite.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(3) # rescale the model

satellite = loas.Satellite( mesh, dt, I0 = I0 )
viewer = loas.Viewer( sat, 30 )

sat.get_parasite_torque = lambda satellite : loas.atmospheric_drag.torque(
    satellite, 50,1, viewer, 1
)

satellite.start()
viewer.run() # retuns only when viewer is closed
satellite.stop()
satellite.join()
