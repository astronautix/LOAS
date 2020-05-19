import numpy as np
import random
import math
import multiprocessing as mp
import scipy.stats
import scipy.constants

import loas
from .torque import Torque


def silent_interrupt(f):
    def f_silent(*args, **kwargs):
        try:
            f(*args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            pass
    return f_silent


@silent_interrupt
def _rayTestingWorker(
    workers_input_queue,
    workers_output_queue,
    create_batch_data_save,
    sat_mesh,
    sat_bs_radius,
    max_part_batch
):
    """
    Unitary worker for Sparse Atmospheric drag computation. Computes the collision of a certain amount of random particles on the mesh, adn sends back the torque

    Every worker is launched only once. The trick is that, once starte, communicate with the rest of the program through Queues.
    There are two queues, one for input, one for output. The worker waits for any input and once it gets it, runs the simulation, and outputs its parameters.

    :param bounding_sphere_radius: Radius of the bounding sphere of the saellite, i.e. the sphere that entirely includes the satellite. It defines the radius of thje circle in witch it is needed to generate particles
    :type bounding_sphere_radius: float
    :param part_mass: Mass of each particles
    :type part_mass: float
    :param dt: Time step of the simulation, i.e. satellite.dt
    :type dt: float
    :param speed: Relative speed of the satellite in the fluid. Can be calculated from the orbital parameters
    :type speed: float
    :param sat_mesh: Satellite's mesh
    :type sat_mesh: trimesh.Trimesh
    :param create_batch_data_save: if set to true, the worker will output at each iteration a list containing the result of every particle simultation so that it can be worked into a batch and printed on the pyglet's window.
    :type create_batch_data_save: bool
    :param workers_input_queue: Queue which is used to pass parameters to the worker
    :type workers_input_queue: multiprocessing.Queue
    :param workers_output_queue: Queue which is used to sends the workers simulation result
    :type workers_input_queue: multiprocessing.Queue
    """

    workers_running = True
    while workers_running:
        (
            workers_running,
            sat_speed,
            sat_Q,
            sat_W,
            part_pending,
            part_mass,
            part_temp_i,
            part_temp_r,
            coll_epsilon,
            dt
        ) = workers_input_queue.get()
        if not workers_running:
            return

        if create_batch_data_save:
            batch_data_save = []
        else:
            batch_data_save = None

        dir_sat = sat_Q.R2V(sat_speed)[:,0]

        def _getRandomOrigin():
            r = sat_bs_radius*math.sqrt(random.random())
            theta = 2*math.pi*random.random()
            return (
                r*math.cos(theta),
                r*math.sin(theta),
                -2*sat_bs_radius
            )

        torque  = loas.utils.vector.tov(0,0,0)
        drag = 0

        while part_pending > 0:
            # disable batching if max_part_batch to 0
            if max_part_batch > 0:
                part_batch = min(part_pending, max_part_batch)
            else:
                part_batch = part_pending
            part_pending -= part_batch

            origins = [_getRandomOrigin() for _ in range(part_batch)]
            origins_sat = np.array([sat_Q.R2V(loas.utils.vector.tov(*origin))[:,0] for origin in origins])
            locations, indexes_ray, indexes_tri = sat_mesh.ray.intersects_location(
                ray_origins=origins_sat,
                ray_directions=[dir_sat]*part_batch
            )

            #filter only for closest point
            locations_filtered = {}
            for index, location in enumerate(locations):
                index_tri = indexes_tri[index]
                index_ray = indexes_ray[index]
                origin_sat = origins_sat[index_ray]
                origin = origins[index_ray]
                dist = (origin_sat[0] - location[0])**2 + (origin_sat[1] - location[1])**2 + (origin_sat[2] - location[2])**2

                if not index_ray in locations_filtered:
                    locations_filtered[index_ray] = (location, index_tri, origin, dist)

                elif locations_filtered[index_ray][3] > dist:
                    locations_filtered[index_ray] = (location, index_tri, origin, dist)

            # process torque given by actual hit point
            for location_sat, index_tri, origin, _ in locations_filtered.values():

                location = sat_Q.V2R(loas.utils.vector.tov(*location_sat))
                normal = sat_Q.V2R(loas.utils.vector.tov(*sat_mesh.face_normals[index_tri]))
                rel_speed = sat_speed - loas.utils.vector.cross(sat_W, location)

                normal_rel_speed = (np.transpose(normal) @ rel_speed)[0,0]
                if normal_rel_speed > 0:
                    # no actual collision
                    continue

                if random.random() < coll_epsilon:
                    # specular reflexion
                    delta_rel_speed = 2 * normal_rel_speed * normal / np.linalg.norm(normal)
                else:
                    # diffuse reflexion
                    normal_refl_speed = abs(scipy.stats.norm.rvs(
                        scale = (scipy.constants.k*part_temp_r/part_mass)**(1/2)
                    ))
                    delta_rel_speed = (normal_rel_speed+normal_refl_speed) * normal / np.linalg.norm(normal)

                momentum = part_mass*delta_rel_speed # elastic collision
                drag += ((np.transpose(sat_speed)/np.linalg.norm(sat_speed)) @ momentum/dt)[0,0]
                torque += loas.utils.vector.cross(location, momentum/dt)

                if create_batch_data_save:
                    batch_data_save.append((origin, location))

        workers_output_queue.put((torque, drag, batch_data_save))


class SparseDrag(Torque):
    """
    Inherits form loas.Torque, defines the algorithms to compute Sparse Atmospheric drag.
    """

    def __init__(
        self,
        satellite,
        sat_speed = 7000,
        sat_temp = 300,
        part_density = 1e-11,
        part_mol_mass = 0.016,
        part_temp = 1800,
        part_per_iteration = 100,
        coll_epsilon = 0.1,
        coll_alpha = 0.95,
        nb_workers = 1,
        max_simultaneous_part = 0,
        output = None,
        output_particle_data = False
    ):
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
        :type viewer: loas.output.Viewer
        """

        super().__init__(satellite)
        self.sat_speed = loas.utils.vector.tov(0,0,sat_speed)
        self.sat_temp = sat_temp
        self.sat_bs_radius = np.linalg.norm(satellite.mesh.extents)/2
        self.part_density = part_density
        self.part_mass = part_mol_mass/scipy.constants.N_A
        self.part_temp_i = part_temp
        self.part_temp_r = part_temp + coll_alpha*(sat_temp - part_temp)
        self.part_per_iteration = part_per_iteration
        self.coll_epsilon = coll_epsilon

        self.scale_factor = part_density / self.part_mass * satellite.dt * sat_speed * math.pi*self.sat_bs_radius**2 /part_per_iteration
        self.nb_workers = nb_workers
        self.workers = []
        self.part_per_batch = int(max_simultaneous_part/nb_workers)
        self.output = output
        self.output_particle_data = output_particle_data
        self.workers_input_queue = mp.Queue()
        self.workers_output_queue = mp.Queue()

    @property
    def coll_alpha(self, alpha):
        raise AttributeError

    @coll_alpha.setter
    def coll_alpha(self, alpha):
        self.part_temp_r = self.part_temp_i + alpha*(self.sat_temp - self.part_temp_i)

    def start(self):
        """
        Starts the workers
        """

        args = (
            self.workers_input_queue,
            self.workers_output_queue,
            self.output_particle_data,
            self.satellite.mesh,
            self.sat_bs_radius,
            self.part_per_batch
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
                False, None, None, None, None, None, None, None, None, None
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

        nb_particles = max(int(round(random.normalvariate(
            mu = self.part_per_iteration,
            sigma = (self.part_per_iteration)**(1/2)
        ))),0) # uniform distribution of particle in an infinite volume

        nb_part = round(nb_particles/self.nb_workers)

        args = (
            True,
            self.sat_speed,
            self.satellite.Q,
            self.satellite.W,
            nb_part,
            self.part_mass,
            self.part_temp_i,
            self.part_temp_r,
            self.coll_epsilon,
            self.satellite.dt
        )
        for i in range(self.nb_workers):
            self.workers_input_queue.put(args)

        torque  = loas.utils.vector.tov(0,0,0)
        drag = 0
        particle_data = []
        for _ in range(self.nb_workers):
            torque_add, drag_add, particle_data_add = self.workers_output_queue.get()
            torque += torque_add
            drag += drag_add
            if particle_data_add is not None:
                particle_data += particle_data_add

        torque *= self.scale_factor
        drag *= self.scale_factor

        self.output.update(
            t = self.satellite.t,
            parasite_drag = drag,
            parasite_torque = torque,
            satellite_speed = self.sat_speed,
            parasite_particle_data = particle_data
        )

        return torque
