import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import loas
import random
import trimesh

###############################
# Paramètres de la simulation #
###############################
# Temps
dt = 1/25 #pas de temps de la simulation
fAffichage = 25 #fréquence d'affichage

# Géométrie
lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
m = 1 #masse du satellite
I0 = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite

# Mouvement
W0 = 0*np.array([[2*(random.random()-0.5)] for i in range(3)]) #rotation initiale dans le référentiel R_r
J = 1

# load mesh object and resize it
mesh = trimesh.load_mesh("bunny.stl")
mesh.apply_transform(np.eye(4)*.02) #resize mesh

ray_origins = np.array([[0, 0, -3],
                        [2, 2, -3]])
ray_directions = np.array([[0, 0, 1],
                           [0, 0, 1]])

locations, index_ray, index_tri = mesh.ray.intersects_location(
    ray_origins=ray_origins,
    ray_directions=ray_directions
)

#################
# Main function #
#################
sim = loas.simulator.Simulator(dt, dw0 = np.array([[1.],[0.],[0.]]), I0 = I0)

viewer = loas.viewer.Viewer( mesh, sim.getQ, 30 )

for i in range(len(ray_origins)):
    viewer.stat_batch.add_line(ray_origins[i], ray_directions[i])
    viewer.stat_batch.add_pyramid(locations[i])


sim.start()
viewer.run()
sim.stop()
sim.join()
