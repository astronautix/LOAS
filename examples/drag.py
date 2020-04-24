import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import trimesh
import loas

dt = 1/25 # Simulation time step
I0 = np.diag((20,20,20)) # Satellite inertia tensor

# load mesh object and resize it
mesh = trimesh.load_mesh("./satellite.stl")
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2) # center the satellite (the mass center should be on 0,0)
mesh.apply_scale(3) # rescale the model

satellite = loas.Satellite( mesh, dt, I0 = I0 )
viewer = loas.Viewer( satellite, 30 )
drag_torque = loas.SparseDrag( satellite, 100, 1, 1, 4, viewer)
satellite.addParasiteTorque( drag_torque )

drag_torque.start()
satellite.start()
viewer.run() # retuns only when viewer is closed

satellite.stop()
drag_torque.stop()
satellite.join()
drag_torque.join()
