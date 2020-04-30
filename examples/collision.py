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

    while satellite.running:
        batch = loas.output.viewer.CustomBatch()
        for particle in particles:
            batch.add_line(particle.origin[:,0], particle.speed[:,0])
            location, normal, rel_speed, momentum = particle.getCollisionOnMesh(satellite)
            if location is not None:
                batch.add_pyramid(location[:,0])
                batch.add_line(location[:,0], normal[:,0])
                batch.add_line(location[:,0], rel_speed[:,0], color=(0,255,0))
                batch.add_line(location[:,0], momentum[:,0], color=(0,0,255))
        viewer.set_named_batch('uopidou',batch)
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
bounds = np.array(mesh.bounds)
mesh.apply_translation(-(bounds[0] + bounds[1])/2)
mesh.apply_translation((0,.1,0))
mesh.apply_scale(.03)

sat = loas.Satellite( mesh, dt, dw0 = np.array([[1.],[0.],[0.]]), I0 = I0 )

viewer = loas.output.Viewer( sat, 30 )

sat.start()
threading.Thread(
    target = lambda: showImpactOnSatellite(
        sat,
        [
            loas.atmospheric_drag.Particle(
                np.array([[i],[0],[-5]]),
                np.array([[0],[0],[1]])
            )
            for i in range(-1,2)
        ],
        viewer
    )
).start()
viewer.run()
sat.stop()
sat.join()
