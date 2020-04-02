import sys
import os
sys.path.append(os.path.join(*['..']*2))
from math import *
import numpy as np
from simulator import Simulator
from quaternion import Quaternion
###############################
# Paramètres de la simulation #
###############################

from conf import *

###################################
# Initialisation de la simulation #
###################################
# Initialisation des variables de simulation
t = 0
nbit = 0
dw = np.array([[1.],[0.],[0.]]) # vecteur de l'accélération angulaire des RW
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
B = np.array([[0],[0],[0]])
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
L0 = np.dot(I,W0)
Wr = []
qs = []

# Simulateur
sim = Simulator(dt,L0)

############################
# Initialisation graphique #
############################


""" 3D model loading: complex architectural model with multiple vertex groups
and images
"""
import math, random

import pi3d

# Setup display and initialise pi3d
DISPLAY = pi3d.Display.create(x=50, y=50, frames_per_second=30)
DISPLAY.set_background(0,0,0,1)    	# r,g,b,alpha
light = pi3d.Light(lightpos=(1.0, 0.0, 0.1))

# load shader
shader = pi3d.Shader("uv_light")
flatsh = pi3d.Shader("uv_flat")

# load model and texture
watimg = pi3d.Texture("water.jpg")
mymodel = pi3d.Model(
  file_string="teapot.obj",
  name="Teapot", sx=1, sy=1, sz=1, z=5.8)

# set model display properties
mymodel.set_shader(shader)
mymodel.set_textures([watimg])

# create the object that will handle keyboard
mykeys = pi3d.Keyboard()

while DISPLAY.loop_running():

    # on récupère le prochain vecteur rotation (on fait ube étape dans la sim)
    Q = sim.getNextIteration(M,dw,J,B,I)

    # Actualisation de l'affichage graphique
    mymodel.rotate_to_direction(Q.V2R(np.array([[0.],[0.],[1.]]))[:,0])
    mymodel.draw()

    #Press ESCAPE to terminate
    k = mykeys.read()
    if k > -1:
        if k == 27:    #Escape key
            mykeys.close()
            DISPLAY.destroy()
            break

    nbit += 1

quit()
