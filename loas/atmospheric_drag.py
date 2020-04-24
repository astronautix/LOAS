import numpy as np
import loas
import trimesh
import random
import math
import multiprocessing as mp

def rayTestingWorker(bounding_sphere_radius, particle_mass, dt, speed, mesh, create_batch_data_save, workers_input_queue, workers_output_queue):

    satellite_attitude = loas.Quaternion(1,0,0,0)
    satellite_rot_speed = loas.vector.tov(0,0,0)
    workers_running = True

    while workers_running:

        pending_particles, satellite_attitude, satellite_rot_speed, workers_running = workers_input_queue.get()
        if not workers_running:
            return

        if create_batch_data_save:
            batch_data_save = []
        else:
            batch_data_save = None

        torque  = loas.vector.tov(0,0,0)
        for _ in range(pending_particles):

            r = bounding_sphere_radius*math.sqrt(random.random())
            theta = 2*math.pi*random.random()
            origin = loas.vector.tov(
                r*math.cos(theta),
                r*math.sin(theta),
                -2*bounding_sphere_radius
            )

            location, _, _, momentum = Particle(origin, speed, particle_mass).getCollisionOnMesh(mesh, satellite_attitude, satellite_rot_speed)

            if location is not None:
                torque += loas.vector.cross(location, momentum/dt)

            if create_batch_data_save:
                batch_data_save.append((origin, location))

        workers_output_queue.put((torque, batch_data_save))


class SparseDrag(loas.Torque):

    def __init__(self, satellite, particle_density, satellite_speed, particle_mass, nb_workers = 1, viewer = None):
        super().__init__(satellite, viewer)
        self.speed = loas.vector.tov(0,0,satellite_speed)
        self.particle_mass = particle_mass
        self.bounding_sphere_radius = np.linalg.norm(satellite.mesh.extents)/2
        self.particle_rate = float(particle_density * satellite.dt*satellite_speed * math.pi*self.bounding_sphere_radius**2)
        self.nb_workers = nb_workers
        self.workers = []
        self.viewer = viewer

        self.workers_input_queue = mp.Queue()
        self.workers_output_queue = mp.Queue()

    def start(self):
        args = (
            self.bounding_sphere_radius,
            self.particle_mass,
            self.satellite.dt,
            self.speed,
            self.satellite.mesh,
            self.viewer is not None,
            self.workers_input_queue,
            self.workers_output_queue
        )
        for _ in range(self.nb_workers):
            worker = mp.Process(target=rayTestingWorker, args=args)
            worker.start()
            self.workers.append(worker)

    def stop(self):
        for _ in range(self.nb_workers):
            self.workers_input_queue.put((
                None,
                None,
                None,
                False
            ))

    def join(self):
        for worker in self.workers:
            worker.join()

    def getTorque(self):
        if self.viewer is not None:
            batch = loas.viewer.CustomBatch()

        nb_particles = max(int(round(random.normalvariate(
            mu = self.particle_rate,
            sigma = (self.particle_rate)**(1/2)
        ))),0) # uniform distribution of particle in an infinite volume

        nb_particles = int(self.particle_rate) #testing

        nb_part = nb_particles//self.nb_workers
        for _ in range(self.nb_workers):
            self.workers_input_queue.put((
                nb_part,
                self.satellite.Q,
                self.satellite.getW(),
                True
            ))

        torque  = loas.vector.tov(0,0,0)
        for _ in range(self.nb_workers):
            torqueadd, batch_data_save = self.workers_output_queue.get()
            torque += torqueadd

            if batch_data_save is not None:
                for origin, location in batch_data_save:
                    batch.add_line(origin[:,0], self.speed[:,0])
                    if location is not None:
                        batch.add_pyramid(location[:,0])

        if self.viewer is not None:
            batch.add_line(origin=(0,0,0), dir=torque[:,0]/100, color=(255,255,255))
            self.viewer.set_named_batch('main_drag', batch)

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

    def getCollisionOnMesh(self, mesh, Q, W):
        """
        Computes the collition coordinate of the particle on the satellite's mesh

        :param satellite: Satellite that has to encounter the particle
        :type satellite: loas.Satellite
        """

        origin_sat = Q.R2V(self.origin)[:,0]
        dir_sat = Q.R2V(self.speed)[:,0]

        locations, index_ray, index_tri = mesh.ray.intersects_location(
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
        normal = mesh.face_normals[index_tri[index_collision]]

        location = Q.V2R(np.array([[i] for i in location]))
        normal = Q.V2R(np.array([[i] for i in normal]))

        rel_speed = self.speed - loas.vector.cross(W, location)

        normal_rel_speed = (np.transpose(normal) @ rel_speed)[0,0]
        if normal_rel_speed > 0:
            normal_rel_speed = 0
        normal_rel_speed_vec = normal_rel_speed * normal / np.linalg.norm(normal)

        momentum = 2*self.mass*normal_rel_speed_vec # elastic collision

        return location, normal, rel_speed, momentum
