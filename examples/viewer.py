import sys
import os
sys.path.append(os.path.join(os.getcwd(), ".."))
import numpy as np
import loas
import random
import trimesh


def viewer():

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
    mesh = trimesh.load_mesh("./bunny.stl")
    mesh.apply_transform(np.eye(4)*.02) #resize mesh

    particles = [
        loas.atmospheric_drag.Particle((.3,0,-3), (0,0,1)),
        loas.atmospheric_drag.Particle((1,0,-3), (0,0,1)),
        loas.atmospheric_drag.Particle((-.3,0,-3), (0,0,1)),
    ]

    sat = loas.Satellite( mesh, dt, dw0 = np.array([[1.],[0.],[0.]]), I0 = I0 )

    viewer = loas.Viewer( sat, 30 )

    for particle in particles:
        viewer.stat_batch.add_line(particle.origin, particle.dir)
        location, normal = particle.getCollisionPointOnMesh(mesh, loas.Quaternion(1,0,0,0))
        viewer.stat_batch.add_pyramid(location)
        viewer.stat_batch.add_line(location, normal)

    sat.start()
    viewer.run()
    sat.stop()
    sat.join()


if __name__ == "__main__":
    viewer()
