import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import loas
import random
import trimesh
import threading
import time

#########################
# Simulation parameters #
#########################
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
mesh = trimesh.load_mesh("./satellite.stl")
val = (2)**(1/2)/2
mesh.apply_transform(
    np.array([
        [0, -1, 0, 0],
        [1, 0, 0, 0],
        [0,0,1,0],
        [0,0,0,1]
    ]) @
    np.array([
        [1,0,0,-28.85],
        [0,1,0,-0.2],
        [0,0,1,-0.1],
        [0,0,0,1]
    ]) @
    np.eye(4)*3
) #resize mesh

sat = loas.Satellite( mesh, dt, I0 = I0 )

viewer = loas.Viewer( sat, 30 )

sat.get_parasite_torque = lambda satellite : loas.atmospheric_drag.atmospheric_drag_torque(satellite, 100,1, viewer, 1, [-2.5,2.5], [-2.5,2.5])

sat.start()
viewer.run()
sat.stop()
sat.join()
