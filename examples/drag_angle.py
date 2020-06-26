import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import trimesh
import loas
import math

# simulation parameters
dt = 1/25 # Simulation time step
I0 = np.diag((200,200,200)) # Satellite inertia tensor

# load mesh object and resize it
mesh = trimesh.load_mesh("./models/slab.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(.08) # rescale the model

drag = loas.rad.RAD(
    sat_mesh = mesh,
    model = loas.rad.models.maxwell(0.10),
    part_per_iteration = 1e4,
    nb_workers = 6
)
drag.start()
sat_Q = [loas.utils.Quaternion(math.cos(angle/2), math.sin(angle/2), 0, 0) for angle in np.linspace(0, math.pi/2, 10)]
print(drag.runSim(
    sat_W = loas.utils.tov(0,0,0),
    sat_Q = sat_Q,
    sat_speed = 7000,
    sat_temp = 300,
    part_density = 1e-11,
    part_mol_mass = 0.016,
    part_temp = 1800,
    with_drag_coef = True
))
drag.stop()
