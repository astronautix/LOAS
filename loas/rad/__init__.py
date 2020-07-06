import numpy as np
import random
import math
import multiprocessing as mp
import scipy.constants
import copy
import warnings

import loas

from . import models

def silent_interrupt(f):
    def f_silent(*args, **kwargs):
        try:
            f(*args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            pass
    return f_silent


@silent_interrupt
def _sparse_drag_worker(
    workers_input_queue,
    workers_output_queue,
    sat_mesh,
    max_part_batch,
    part_per_iteration,
    model
):
    """
    Unitary worker for Sparse Atmospheric drag computation. Computes the collision of a certain amount of random particles on the mesh, adn sends back the torque

    Every worker is launched only once. The trick is that, once started, communicate with the rest of the program through Queues.
    There are two queues, one for input, one for output. The worker waits for any input and once it gets it, runs the simulation, and outputs its parameters.

    :param workers_input_queue: Queue which is used to pass parameters to the worker
    :type workers_input_queue: multiprocessing.Queue
    :param workers_output_queue: Queue which is used to sends the workers simulation result
    :type workers_input_queue: multiprocessing.Queue
    :param create_batch_data_save: if set to true, the worker will output at each iteration a list containing the result of every particle simultation so that it can be worked into a batch and printed on the pyglet's window.
    :type create_batch_data_save: bool
    :param sat_mesh: Satellite's mesh
    :type sat_mesh: trimesh.Trimesh
    :param sat_bs_radius: Radius of the bounding sphere of the saellite, i.e. the sphere that entirely includes the satellite. It defines the radius of thje circle in witch it is needed to generate particles
    :type sat_bs_radius: float
    :param max_part_batch: Maximum number of particles given at once to the ray tester. If set to 0, disables, the limit.
    :param max_part_batch: int
    """

    sat_bs_radius = sat_mesh.bounding_sphere.extents[0]/2
    sat_bs_center = loas.utils.tov(*sat_mesh.bounding_sphere.center_mass)
    workers_running = True

    def _getRandomOrigin():
        r = sat_bs_radius*math.sqrt(random.random())
        theta = 2*math.pi*random.random()
        rel_pos = loas.utils.tov(
            r*math.cos(theta),
            r*math.sin(theta),
            -2*sat_bs_radius
        )
        return rel_pos + sat_bs_center

    while workers_running:
        args = (
            workers_running,
            sat_speed,
            sat_Q,
            sat_W,
            sat_temp,
            part_pending,
            part_mass,
            part_temp,
            part_density
        ) = workers_input_queue.get()
        if not workers_running:
            return

        sat_speed = loas.utils.tov(0,0,sat_speed)
        dir_sat = sat_Q.R2V(sat_speed)[:,0]
        torque_dt  = loas.utils.tov(0,0,0)
        drag_dt = 0

        while part_pending > 0:
            # disable batching if max_part_batch to 0
            if max_part_batch > 0:
                part_batch = min(part_pending, max_part_batch)
            else:
                part_batch = part_pending
            part_pending -= part_batch

            origins_sat = np.array([sat_Q.R2V(_getRandomOrigin())[:,0] for _ in range(part_batch)])
            locations, indexes_ray, indexes_tri = sat_mesh.ray.intersects_location(
                ray_origins=origins_sat,
                ray_directions=[dir_sat]*part_batch
            )

            #filter only for closest point
            locations_filtered = {}
            for index, location_sat in enumerate(locations):
                index_tri = indexes_tri[index]
                index_ray = indexes_ray[index]
                origin_sat = origins_sat[index_ray]
                dist = (origin_sat[0] - location_sat[0])**2 + (origin_sat[1] - location_sat[1])**2 + (origin_sat[2] - location_sat[2])**2

                if not index_ray in locations_filtered:
                    locations_filtered[index_ray] = (location_sat, index_tri, dist)
                elif locations_filtered[index_ray][2] > dist:
                    locations_filtered[index_ray] = (location_sat, index_tri, dist)

            # process torque given by actual hit point
            for location_sat, index_tri, _ in locations_filtered.values():
                location = sat_Q.V2R(loas.utils.tov(*location_sat))
                normal = sat_Q.V2R(loas.utils.tov(*sat_mesh.face_normals[index_tri]))
                normal /= np.linalg.norm(normal)
                part_speed_i = sat_speed - loas.utils.cross(sat_W, location)
                part_speed_r = model(part_speed_i, normal, sat_temp, part_mass)
                momentum = part_mass*(part_speed_i-part_speed_r)
                drag_dt += ((np.transpose(sat_speed)/np.linalg.norm(sat_speed)) @ momentum)[0,0]
                torque_dt += loas.utils.cross(location, momentum)

        scale_factor = part_density / part_mass * sat_speed * math.pi*sat_bs_radius**2 / part_per_iteration
        torque = torque_dt * scale_factor
        drag = drag_dt * scale_factor

        workers_output_queue.put((torque, drag))


class RAD():
    """
    Inherits form loas.Torque, defines the algorithms to compute Sparse Atmospheric drag.
    """

    def __init__(
        self,
        sat_mesh,
        model,
        part_per_iteration = 100,
        nb_workers = 1,
        max_simultaneous_part = 0
    ):
        """
        :param satellite: Satellite instance that represents simulation
        :type satellite: loas.Satellite
        :param sat_speed: Satellite speed relative to the ionosphere
        :param sat_speed: float
        :param sat_temp: Satellite temperature
        :param sat_temp: float
        :param part_density: Particle density
        :type part_density: int
        :param part_mol_mass: Molar mass of the particles
        :type part_mol_mass: float
        :param part_temp: Temperature of the particles
        :type part_temp: float
        :param part_per_iteration: Average number of particles simulated at each iteration
        :type part_per_iteration: int
        :param coll_epsilon: Ratio of specular reflexion
        :type coll_epsilon: float
        :param coll_alpha: Accomodation coefficient
        :type coll_alpha: float
        :param model_type: Type of the reflexion model used. 0 : Kinetic model, 1 : Semi-thermal model, 2 : Full-thermal model
        :param nb_workers: Number of parallels workers (thus processes) that are launched for the simulation
        :type nb_workers: int
        :param max_simultaneous_part: Maximum number of particles simulated simultaneously. It can be used to reduce the RAM usage, but it is detrimental to execution speed. If set to 0, the limit is disabled
        :param max_simultaneous_part: int
        :param output: Type of output to send simulation data. If set to None, it will output nothing
        :type output: loas.output.Output
        :param output_particle_data: If set to True, the simulation will send the origin point and collision point of every particle. It can lead to big ram usage.
        :type output_particle_data: bool
        """
        self.sat_mesh = sat_mesh
        self.model = model

        self.nb_workers = nb_workers
        self.workers = []
        self.part_per_batch = int(max_simultaneous_part/nb_workers)
        self.part_per_iteration = part_per_iteration
        self.workers_input_queue = mp.Queue()
        self.workers_output_queue = mp.Queue()


    def start(self):
        """
        Starts the workers
        """
        args = (
            self.workers_input_queue,
            self.workers_output_queue,
            self.sat_mesh,
            self.part_per_batch,
            self.part_per_iteration,
            self.model
        )

        for _ in range(self.nb_workers):
            worker = mp.Process(target=_sparse_drag_worker, args=args)
            worker.start()
            self.workers.append(worker)

    def stop(self):
        """
        Stops the workers
        """

        for _ in range(self.nb_workers):
            self.workers_input_queue.put((
                False, None, None, None, None, None, None, None, None
            ))

    def join(self):
        """
        Same effect Thread api : waits until every worker returns
        """

        for worker in self.workers:
            worker.join()
        self.workers = []

    def runSim(
        self,
        sat_W,
        sat_Q,
        sat_speed = 7000,
        sat_temp = 300,
        part_density = 1e-11,
        part_mol_mass = 0.016,
        part_temp = 1800,
        with_drag_coef = False
    ):

        kwargs = locals()
        kwargs.pop('self')

        auto_start = False
        if len(self.workers) == 0:
            warnings.warn("The workers have not been started, I am starting them... (You might want to manually start the workers if calling repeatedly runSim to improve performances)")
            auto_start = True
            self.start()

        for key, value in kwargs.items():
            if isinstance(value, list):
                temp_kwargs = copy.deepcopy(kwargs)
                res = []
                for i in value:
                    assert not isinstance(i, list)
                    temp_kwargs[key] = i
                    res.append(self.runSim(**temp_kwargs))
                return res

        if auto_start:
            print("Stopping the workers...")
            self.stop()

        return self._runSingleSim(**kwargs)

    def _runSingleSim(self, sat_W, sat_Q, sat_speed, sat_temp, part_density, part_mol_mass, part_temp, with_drag_coef):

        part_mass = part_mol_mass/scipy.constants.N_A
        nb_part = round(self.part_per_iteration/self.nb_workers)

        args = (
            True,
            sat_speed,
            sat_Q,
            sat_W,
            sat_temp,
            nb_part,
            part_mass,
            part_temp,
            part_density
        )
        for i in range(self.nb_workers):
            self.workers_input_queue.put(args)

        torque  = loas.utils.tov(0,0,0)
        drag = 0
        particle_data = []
        for _ in range(self.nb_workers):
            torque_add, drag_add = self.workers_output_queue.get()
            torque += torque_add
            drag += drag_add

        if with_drag_coef:
            S_p = loas.utils.projected_area(
                self.sat_mesh,
                sat_Q.R2V(loas.utils.tov(0,0,1))
            )
            drag_coef = drag*2/part_density/sat_speed**2/S_p
            return drag, torque, drag_coef

        return drag, torque

    def getTorque(self):
        return self.runSim()[1]
