import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import trimesh
import loas
import math
import threading

# simulation parameters
dt = 1/25 # Simulation time step
I0 = np.diag((200,200,200)) # Satellite inertia tensor
part_density = 10**(-11) #kg.m-3
part_mol_mass = 16e-3 #atomic oxygen
part_per_iterations = 1e5
sat_speed = 7000
part_mass = part_mol_mass / 6.022e23

def processAngle(satellite, store, viewer):
    for angle in np.linspace(0, math.pi/2, 100):
        print(angle)
        satellite.Q = loas.utils.Quaternion(math.cos(angle/2), math.sin(angle/2), 0, 0)
        satellite.L = loas.utils.vector.tov(0,0,0)
        satellite.iterate()
        store.update(0,quaternion = satellite.Q)
        #viewer.update(0, satellite = satellite) # force viewer update

# load mesh object and resize it
mesh = trimesh.load_mesh("./models/satellite.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(3) # rescale the model

# define main objects
viewer =  loas.output.Viewer( mesh )
store = loas.output.Store()
satellite = loas.Satellite( mesh, dt, I0 = I0, output=viewer)
drag_torque = loas.parasite.SparseDrag( satellite, part_density, sat_speed, part_mass, part_per_iterations, 6, output=store)
satellite.addParasiteTorque( drag_torque )

drag_torque.start()
threading.Thread(target = processAngle, args=(satellite, store, viewer)).start()
viewer.run()
