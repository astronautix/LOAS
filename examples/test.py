import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import loas
import random
import trimesh
import threading
import time

def showImpactOnSatellite(satellite, particles, viewer):
    """
    Is launched in a thread, process the impact
    position of the set of *particles* on the *satellite*,
    and display it in the *viewer*
    """

    oldbatch = loas.viewer.CustomBatch()
    while 1:
        batch = loas.viewer.CustomBatch()
        for particle in particles:
            batch.add_line(particle.origin[:,0], particle.dir[:,0])
            location, normal = particle.getCollisionPointOnMesh(satellite)
            if location is not None:
                batch.add_pyramid(location)
                batch.add_line(location, normal)
        oldbatch.hide()
        viewer.add_batch(batch)
        oldbatch = batch
        time.sleep(.1)

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
mesh = trimesh.load_mesh("./bunny.stl")
mesh.apply_transform(np.eye(4)*.02) #resize mesh

sat = loas.Satellite( mesh, dt, dw0 = np.array([[1.],[0.],[0.]]), I0 = I0 )

viewer = loas.Viewer( sat, 30 )

threading.Thread(
    target = lambda: showImpactOnSatellite(
        sat,
        [
            loas.atmospheric_drag.Particle(np.array([[.3],[0],[-5]]), np.array([[0],[0],[1]])),
            loas.atmospheric_drag.Particle(np.array([[1],[0],[-5]]), np.array([[0],[0],[1]])),
            loas.atmospheric_drag.Particle(np.array([[-.3],[0],[-5]]), np.array([[0],[0],[1]])),
        ],
        viewer
    )
).start()
sat.start()
viewer.run()
sat.stop()
sat.join()
