import numpy as np
import random
import math
import scipy.stats
import scipy.constants

import loas

def get_Q_sfc(normal):
    """
    Returns the quaternion to pass from the reference frame to
    the local surface frame where the x axis is normal to the surface
    """
    dir_rot = normal + loas.Vec(1,0,0)
    if np.linalg.norm(dir_rot) < 1e-6:
        dir_rot = loas.Vec(0,1,0)
    return loas.Quat(0, *dir_rot)

def maxwell(epsilon):
    """
    Maxwell reflexion model
    :param epsilon: Specular coefficient
    :type epsilon: float
    """
    # See Sharipov, Rarefied gas dynamics, 4
    def model(part_speed_i, normal, sat_temp, part_mass):
        Q_sfc = get_Q_sfc(normal) #quaternion de passage sur la surface
        if random.random() < epsilon:
            # specular reflexion
            normal_rel_speed = (np.transpose(normal) @ part_speed_i)[0,0]*normal
            part_speed_r = part_speed_i - 2*normal_rel_speed
        else:
            part_speed_r_norm = scipy.stats.maxwell.rvs(scale = math.sqrt(scipy.constants.k*sat_temp/part_mass))
            theta = math.asin(2*random.random()-1) #angle par rapport à la normale à la surface (donc le vecteur (1,0,0)) dans le repère de la sfc
            phi = 2*math.pi*random.random() #angle dans le plan (yOz)
            part_speed_r = Q_sfc.V2R(
                part_speed_r_norm*
                loas.Vec(
                    math.cos(theta),
                    math.sin(theta)*math.cos(phi),
                    math.sin(theta)*math.sin(phi)
                )
            )
        return part_speed_r
    return model

def schamberg(theta_, theta_i):
    """
    /!\ UNTESTED /!\
    Schamberg reflexion model
    """
    def model(part_speed_i, normal, sat_temp, part_mass):
        # Schamberg model
        Q_sfc = get_Q_sfc(normal)
        part_speed_r_norm = scipy.stats.maxwell.rvs(scale = math.sqrt(scipy.constants.k*sat_temp/part_mass))
        theta = 2*theta_0/math.pi*math.asin(2*random.random()-1) + theta_i #angle par rapport à la normale à la surface (donc le vecteur (1,0,0)) dans le repère de la sfc
        phi = 2*math.pi*random.random() #angle dans le plan (yOz)
        part_speed_r = Q_sfc.V2R(
            part_speed_r_norm*
            loas.Vec(
                math.cos(theta),
                math.sin(theta)*math.cos(phi),
                math.sin(theta)*math.sin(phi)
            )
        )
        return part_speed_r
    return model

def schamberg_compose(schambergs, coefs):
    """
    /!\ UNTESTED /!\
    Reflexion model made from the linear composition of several schamberg models
    :param schambergs: List of schamberg reflexion models
    :type schambergs: list
    :param coefs: List of the coefficients
    :type coefs: list of floats
    """
    def model(*args, **kwargs):
        used_schamberg = random.choice(schambergs, coefs)
        return used_schamberg(*args,**kwargs)
    return model
