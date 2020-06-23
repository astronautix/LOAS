import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import trimesh
import loas

# simulation parameters
dt = 1/25 # Simulation time step
I0 = np.diag((200,200,200)) # Satellite inertia tensor

# load mesh object and resize it
mesh = trimesh.load_mesh("./models/satellite.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(3) # rescale the model

# define main objects
output = loas.output.Viewer( mesh ) #loas.output.Plotter()
satellite = loas.Satellite( mesh, dt, I0 = I0, output=output)
drag_torque = loas.rad.RAD(
    satellite,
    model = loas.rad.models.maxwell(0.10),
    sat_speed = 7000,
    sat_temp = 300,
    part_density = 1e-11,
    part_mol_mass = 0.016,
    part_temp = 1800,
    part_per_iteration = 1e4,
    nb_workers = 6,
)
satellite.addParasiteTorque( drag_torque )

drag_torque.start() # starts the workers that computes the drag torque
satellite.start() # starts the satellite's simulation

output.run() # retuns only when window is closed

satellite.stop() # stops the satellite
drag_torque.stop() # stop drag_torque workers
satellite.join()
drag_torque.join()
