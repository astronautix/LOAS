import numpy as np
import loas
import trimesh
import random

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


def atmospheric_drag_torque(satellite, particle_rate, speed, viewer, mass_particle, x_domain, y_domain):
    z0 = -5
    dt = satellite.dt
    nb_particles = max(round(random.normalvariate(
        mu = particle_rate,
        sigma = (particle_rate)**(1/2)
    )),0) # uniform distribution of particle in an infinite volume
    torque = loas.vector.tov(0,0,0)
    batch = loas.viewer.CustomBatch()
    for _ in range(nb_particles):
        origin = loas.vector.tov(random.uniform(*x_domain), random.uniform(*y_domain), z0)
        speed_vec = loas.vector.tov(0,0,speed)
        batch.add_line(origin[:,0], speed_vec[:,0])
        particle = Particle(origin, speed_vec, mass_particle)
        location, _, _, momentum = particle.getCollisionOnMesh(satellite)
        if location is None:
            continue
        torque += loas.vector.cross(location, momentum/dt)
        batch.add_pyramid(location[:,0])
    batch.add_line(origin=(0,0,0), dir=(torque*dt)[:,0], color=(255,255,255))
    viewer.set_named_batch('atm_drag_viewer', batch)
    return torque
