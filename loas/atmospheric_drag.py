import numpy as np
import loas
import trimesh

class Particle:
    """
    Represent a single (air, or whatever) particle that will (or not) collide with the satellite
    """

    def __init__(self, origin, dir):
        """
        :param origin: Origin of the particle
        :type origin: (3,) float
        :param dir: Direction of the particle
        :type dir: (3, float)
        """
        self.origin = origin
        self.dir = dir

    def getCollisionPointOnMesh(self, satellite):
        """
        Computes the collition coordinate of the particle on the satellite's mesh

        :param satellite: Satellite that has to encounter the particle
        :type satellite: loas.Satellite
        """

        locations, index_ray, index_tri = satellite.mesh.ray.intersects_location(
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

    def getMomentumFromCollision(self, satellite):
        """
        Computes the momentum delta of the particle due to the collision on the satellite's mesh

        :param satellite: Satellite that has to encounter the particle
        :type satellite: loas.Satellite
        """
        pass

class AtmosphericDrag:
    """
    TODO
    """
    def __init__(self, satellite):
        self.mesh = mesh

    def _getRandomParticle():
        pass
