import numpy as np
import loas
import trimesh
import random
import math
import multiprocessing as mp


def _getCollisionOnMesh(origin, mass, speed, sat_mesh, sat_Q, sat_W):
    """
    Computes the collition coordinate of the particle on the satellite's mesh

    :param sat_mesh: Mesh of the satellite that has to encounter the particle
    :type sat_mesh: trimesh.Trimesh
    :param sat_Q: Quaternion of the satellite
    :type sat_Q: loas.Quaternion
    :para sat_W: Rotation verctor of the satellite
    :type sat_W: (3,1) numpy.array
    """

    origin_sat = sat_Q.R2V(origin)[:,0]
    dir_sat = sat_Q.R2V(speed)[:,0]

    locations, index_ray, index_tri = sat_mesh.ray.intersects_location(
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
    normal = sat_mesh.face_normals[index_tri[index_collision]]

    location = sat_Q.V2R(np.array([[i] for i in location]))
    normal = sat_Q.V2R(np.array([[i] for i in normal]))

    rel_speed = speed - loas.vector.cross(sat_W, location)

    normal_rel_speed = (np.transpose(normal) @ rel_speed)[0,0]
    if normal_rel_speed > 0:
        normal_rel_speed = 0
    normal_rel_speed_vec = normal_rel_speed * normal / np.linalg.norm(normal)

    momentum = 2*mass*normal_rel_speed_vec # elastic collision

    return location, normal, rel_speed, momentum


def _rayTestingWorker(bounding_sphere_radius, particle_mass, dt, speed, mesh, create_batch_data_save, workers_input_queue, workers_output_queue):
    """
    Unitary worker for Sparse Atmospheric drag computation. Computes the collision of a certain amount of random particles on the mesh, adn sends back the torque

    Every worker is launched only once. The trick is that, once starte, communicate with the rest of the program through Queues.
    There are two queues, one for input, one for output. The worker waits for any input and once it gets it, runs the simulation, and outputs its parameters.

    :param bounding_sphere_radius: Radius of the bounding sphere of the saellite, i.e. the sphere that entirely includes the satellite. It defines the radius of thje circle in witch it is needed to generate particles
    :type bounding_sphere_radius: float
    :param particle_mass: Mass of each particles
    :type particle_mass: float
    :param dt: Time step of the simulation, i.e. satellite.dt
    :type dt: float
    :param speed: Relative speed of the satellite in the fluid. Can be calculated from the orbital parameters
    :type speed: float
    :param mesh: Satellite's mesh
    :type mesh: trimesh.Trimesh
    :param create_batch_data_save: if set to true, the worker will output at each iteration a list containing the result of every particle simultation so that it can be worked into a batch and printed on the pyglet's window.
    :type create_batch_data_save: bool
    :param workers_input_queue: Queue which is used to pass parameters to the worker
    :type workers_input_queue: multiprocessing.Queue
    :param workers_output_queue: Queue which is used to sends the workers simulation result
    :type workers_input_queue: multiprocessing.Queue
    """


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

            location, _, _, momentum = _getCollisionOnMesh(origin, particle_mass, speed, mesh, satellite_attitude, satellite_rot_speed)

            if location is not None:
                torque += loas.vector.cross(location, momentum/dt)

            if create_batch_data_save:
                batch_data_save.append((origin, location))

        workers_output_queue.put((torque, batch_data_save))


class SparseDrag(loas.Torque):
    """
    Inherits form loas.Torque, defines the algorithms to compute Sparse Atmospheric drag.
    """

    def __init__(self, satellite, particle_density, satellite_speed, particle_mass, nb_workers = 1, viewer = None):
        """
        :param satellite: Satellite instance that represents simulation
        :type satellite: loas.Satellite
        :param particle_density: Density of particle to be simulated
        :type particle_density: int
        :param particle_mass: Mass of the particles
        :type particle_mass: float
        :param nb_workers: Number of parallels workers (thus threads) that are launched for the simulation
        :type nb_workers: int
        :param viewer: Viewer instance. Will be used to print particle impact on mesh and computed torque. If set to None (or unset), nothing will be displayed.
        :type viewer: loas.Viewer
        """

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
        """
        Starts the workers
        """

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
            worker = mp.Process(target=_rayTestingWorker, args=args)
            worker.start()
            self.workers.append(worker)

    def stop(self):
        """
        Stops the workers
        """

        for _ in range(self.nb_workers):
            self.workers_input_queue.put((
                None,
                None,
                None,
                False
            ))

    def join(self):
        """
        Same effect Thread api : waits until every worker returns
        """

        for worker in self.workers:
            worker.join()

    def getTorque(self):
        """
        Get the torque computed by the class
        """

        if len(self.workers) == 0:
            raise RuntimeError("No workers are running! Call start() method beforehand")

        if self.viewer is not None:
            batch = loas.viewer.CustomBatch()

        nb_particles = max(int(round(random.normalvariate(
            mu = self.particle_rate,
            sigma = (self.particle_rate)**(1/2)
        ))),0) # uniform distribution of particle in an infinite volume


        nb_part = round(nb_particles/self.nb_workers)
        remaining_part = nb_particles - (self.nb_workers-1)*nb_part

        for i in range(self.nb_workers):
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
