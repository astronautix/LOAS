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
real_density = 10**(-11) #kg.m-3
part_per_iterations = 100
sat_speed = 7000

# load mesh object and resize it
mesh = trimesh.load_mesh("./models/sphere.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(.03) # rescale the model

# determine computation values
comp_number_density = part_per_iterations / (dt*sat_speed * math.pi * (np.linalg.norm(mesh.extents)/2)**2)
comp_part_mass = real_density / comp_number_density

# define main objects
output = loas.output.Viewer( mesh ) #loas.output.Plotter()
satellite = loas.Satellite( mesh, dt, I0 = I0, output=output)
drag_torque = loas.parasite.SparseDrag( satellite, comp_number_density, sat_speed, comp_part_mass, 6, output=output)
satellite.addParasiteTorque( drag_torque )

drag_torque.start() # starts the workers that computes the drag torque
satellite.start() # starts the satellite's simulation

output.run() # retuns only when window is closed

satellite.stop() # stops the satellite
drag_torque.stop() # stop drag_torque workers
satellite.join()
drag_torque.join()
