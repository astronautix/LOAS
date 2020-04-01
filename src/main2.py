import sys
import os
sys.path.append(os.path.join(*['..']*2))
from math import *
import numpy as np
from simulator import Simulator
import matplotlib.pyplot as plt
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
    W = sim.getNextIteration(M,dw,J,B,I)

    #affichage de données toute les 10 itérations
    if nbit%10 == 0:
      print("W :", str(W[:,0]), "|| norm :", str(np.linalg.norm(W)), "|| dw :", str(dw[:,0]), "|| B :", str(B[:,0]), "|| Q :", str(sim.Q.axis()[:,0]), "|| M :", str(np.linalg.norm(M)))

    # Actualisation de l'affichage graphique
    b_vector.axis = 1e6*vp.vector(B[0][0],B[1][0],B[2][0])
    satellite.rotate(angle=np.linalg.norm(W)*dt, axis=vp.vector(W[0][0],W[1][0],W[2][0]), origin=vp.vector(10,10,10))

    # Rate : réalise 25 fois la boucle par seconde
    vp.rate(fAffichage) #vp.rate(1/dt)

    mymodel.rotate_to_direction(list(*Q.vec()[:,0]))
    mymodel.draw()

    #Press ESCAPE to terminate
    k = mykeys.read()
    if k > -1:
        if k == 27:    #Escape key
            mykeys.close()
            mymouse.stop()
            DISPLAY.destroy()
            break

    nbit += 1

quit()
