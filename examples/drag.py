import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import trimesh
import loas

# simulation parameters
dt = 1/25 # Simulation time step
I0 = np.diag((200,200,200)) # Satellite inertia tensor
part_density = 10**(-11) #kg.m-3
part_mol_mass = 16e-3 #atomic oxygen
part_per_iterations = 100
sat_speed = 7000
part_mass = part_mol_mass / 6.022e23

# load mesh object and resize it
mesh = trimesh.load_mesh("./models/sphere.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(.03) # rescale the model

# define main objects
output = loas.output.Viewer( mesh ) #loas.output.Plotter()
satellite = loas.Satellite( mesh, dt, I0 = I0, output=output)
drag_torque = loas.parasite.SparseDrag( satellite, part_density, sat_speed, part_mass, part_per_iterations, 6, output=output)
satellite.addParasiteTorque( drag_torque )

drag_torque.start() # starts the workers that computes the drag torque
satellite.start() # starts the satellite's simulation

output.run() # retuns only when window is closed

satellite.stop() # stops the satellite
drag_torque.stop() # stop drag_torque workers
satellite.join()
drag_torque.join()
