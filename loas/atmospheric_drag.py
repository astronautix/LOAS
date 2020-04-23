import numpy as np
import loas
import trimesh
import random
import math

class SparseDrag(loas.Torque):
    def __init__(self, satellite, particle_density, satellite_speed, particle_mass, viewer = None):
        super().__init__(satellite, viewer)
        self.speed = loas.vector.tov(0,0,satellite_speed)
        self.particle_mass = particle_mass
        self.bounding_sphere_radius = np.linalg.norm(satellite.mesh.extents)/2
        self.particle_rate = particle_density * satellite.dt*satellite_speed * math.pi*self.bounding_sphere_radius**2

    def getTorque(self):
        nb_particles = max(int(round(random.normalvariate(
            mu = self.particle_rate,
            sigma = (self.particle_rate)**(1/2)
        ))),0) # uniform distribution of particle in an infinite volume

        torque = loas.vector.tov(0,0,0)

        if self.viewer is not None:
            batch = loas.viewer.CustomBatch()

        for _ in range(nb_particles):

            r = self.bounding_sphere_radius*math.sqrt(random.random())
            theta = 2*math.pi*random.random()
            origin = loas.vector.tov(
                r*math.cos(theta),
                r*math.sin(theta),
                -2*self.bounding_sphere_radius
            )

            location, _, _, momentum = Particle(origin, self.speed, self.particle_mass).getCollisionOnMesh(self.satellite)

            if self.viewer is not None:
                batch.add_line(origin[:,0], self.speed[:,0])
            if location is not None:
                torque += loas.vector.cross(location, momentum/self.satellite.dt)
                if self.viewer is not None:
                    batch.add_pyramid(location[:,0])

        if self.viewer is not None:
            batch.add_line(origin=(0,0,0), dir=torque[:,0]/100, color=(255,255,255))
            self.viewer.set_named_batch('atm_drag_viewer', batch)

        return torque


class Particle:
    """
    Represent a single (air, or whatever) particle that will (or not) collide with the satellite
    """

    def __init__(self, origin, speed, mass=1):
        """
        :param origin: Origin of the particle
        :type origin: (3,) float
        :param dir: Direction of the particle
        :type dir: (3, float)
        """
        self.origin = origin
        self.speed = speed
        self.mass = mass

    def getCollisionOnMesh(self, satellite):
        """
        Computes the collition coordinate of the particle on the satellite's mesh

        :param satellite: Satellite that has to encounter the particle
        :type satellite: loas.Satellite
        """

        origin_sat = satellite.Q.R2V(self.origin)[:,0]
        dir_sat = satellite.Q.R2V(self.speed)[:,0]

        locations, index_ray, index_tri = satellite.mesh.ray.intersects_location(
            ray_origins=[origin_sat],
            ray_directions=[dir_sat]
        )

        if len(locations) == 0:
            return None, None, None, None

        dists = np.array([
            (origin_sat[0] - location[0])**2 +
            (origin_sat[1] - location[1])**2 +
            (origin_sat[2] - location[2])**2
            for location in locations
        ])

        index_collision = np.argmin(dists)

        location = locations[index_collision]
        normal = satellite.mesh.face_normals[index_tri[index_collision]]

        location = satellite.Q.V2R(np.array([[i] for i in location]))
        normal = satellite.Q.V2R(np.array([[i] for i in normal]))

        rel_speed = self.speed - loas.vector.cross(satellite.getW(), location)

        normal_rel_speed = (np.transpose(normal) @ rel_speed)[0,0]
        if normal_rel_speed > 0:
            normal_rel_speed = 0
        normal_rel_speed_vec = normal_rel_speed * normal / np.linalg.norm(normal)

        momentum = 2*self.mass*normal_rel_speed_vec # elastic collision

        return location, normal, rel_speed, momentum
