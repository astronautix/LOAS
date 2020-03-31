import sys
import os
sys.path.append(os.path.join(*['..']*2))
import vpython as vp
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
ux = vp.vector(1,0,0)
uy = vp.vector(0,1,0)
uz = vp.vector(0,0,1)
# trièdre (z,x,y)
axe_x_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*ux, shaftwidth=0.5, color=vp.vector(1,0,0))
axe_y_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*uy, shaftwidth=0.5, color=vp.vector(0,1,0))
axe_z_r = vp.arrow(pos=vp.vector(0,0,0), axis=10*uz, shaftwidth=0.5, color=vp.vector(0,0,1))
#création du satellite avec son repère propre
axe_x_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*ux, shaftwidth=0.1, color=vp.vector(1,0,0))
axe_y_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*uy, shaftwidth=0.1, color=vp.vector(0,1,0))
axe_z_s = vp.arrow(pos=vp.vector(10,10,10), axis=10*uz, shaftwidth=0.1, color=vp.vector(0,0,1))
sugarbox = vp.box(pos=vp.vector(10,10,10), size=vp.vector(lx,ly,lz), axis=vp.vector(0,0,0), up = uy)
satellite = vp.compound([axe_x_s,axe_y_s,axe_z_s,sugarbox])
#vecteur champ B
b_vector = vp.arrow(pos=vp.vector(-5,-5,-5), axis=10*vp.vector(B[0][0],B[1][0],B[2][0]), shaftwidth=0.1, color=vp.vector(1,1,1))

#####################
# Boucle principale #
#####################
while True:

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
    nbit += 1
    t+=dt
