import numpy as np
import loas
import trimesh

class Particle:
    def __init__(self, origin, dir):
        self.origin = origin
        self.dir = dir

    def getCollisionPointOnMesh(self, mesh, mesh_Q):
        locations, index_ray, index_tri = mesh.ray.intersects_location(
            ray_origins=[self.origin],
            ray_directions=[self.dir]
        )

        dists = np.array([
            (self.origin[0] - location[0])**2 +
            (self.origin[1] - location[1])**2 +
            (self.origin[2] - location[2])**2
            for location in locations
        ])

        index_collision = np.argmin(dists)

        location = locations[index_collision]
        normal = mesh.face_normals[index_tri[index_collision]]

        return location, normal

    def getMomentumFromCollision(self, mesh, mesh_Q, mesh_W):
        pass

class AtmosphericDrag:
    def __init__(self, satellite):
        self.mesh = mesh

    def _getRandomParticle():
        pass
