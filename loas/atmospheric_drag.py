import numpy as np
import loas
import trimesh

class Particle:
    """
    Represent a single (air, or whatever) particle that will (or not) collide with the satellite
    """

    def __init__(self, origin, dir, mass=1):
        """
        :param origin: Origin of the particle
        :type origin: (3,) float
        :param dir: Direction of the particle
        :type dir: (3, float)
        """
        self.origin = origin
        self.dir = dir
        self.mass = mass

    def getCollisionPointOnMesh(self, satellite):
        """
        Computes the collition coordinate of the particle on the satellite's mesh

        :param satellite: Satellite that has to encounter the particle
        :type satellite: loas.Satellite
        """

        origin_sat = satellite.Q.R2V(self.origin)[:,0]
        dir_sat = satellite.Q.R2V(self.dir)[:,0]

        locations, index_ray, index_tri = satellite.mesh.ray.intersects_location(
            ray_origins=[origin_sat],
            ray_directions=[dir_sat]
        )

        if len(locations) == 0:
            return None, None

        dists = np.array([
            (origin_sat[0] - location[0])**2 +
            (origin_sat[1] - location[1])**2 +
            (origin_sat[2] - location[2])**2
            for location in locations
        ])

        index_collision = np.argmin(dists)

        location = locations[index_collision]
        normal = satellite.mesh.face_normals[index_tri[index_collision]]

        location = satellite.Q.V2R(np.array([[i] for i in location]))[:,0]
        normal = satellite.Q.V2R(np.array([[i] for i in normal]))[:,0]

        return location, normal

    def getMomentumFromCollision(self, satellite):
        """
        Computes the momentum delta of the particle due to the collision on the satellite's mesh

        :param satellite: Satellite that has to encounter the particle
        :type satellite: loas.Satellite
        """

        pass
