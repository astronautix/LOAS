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

# define main objects
store = loas.output.Store()
satellite = loas.Satellite( mesh, dt, I0 = I0, output=None)
drag_torque = loas.parasite.SparseDrag(
    satellite,
    sat_speed = 7000,
    sat_temp = 300,
    part_density = 1e-11,
    part_mol_mass = 0.016,
    part_temp = 1800,
    part_per_iteration = 1e4,
    coll_epsilon = 0.10,
    coll_alpha = 0.95,
    nb_workers = 6,
    output=store
)
satellite.addParasiteTorque( drag_torque )

drag_torque.start()
for angle in np.linspace(0, math.pi/2, 10):
    print(angle)
    satellite.Q = loas.utils.Quaternion(math.cos(angle/2), math.sin(angle/2), 0, 0)
    satellite.L = loas.utils.vector.tov(0,0,0)
    satellite.iterate()
    store.update(0,quaternion = satellite.Q)
drag_torque.stop()
drag_torque.join()

print(np.array(store['parasite_drag'])[:,1]*2/1e-11/16.32/7000**2)
