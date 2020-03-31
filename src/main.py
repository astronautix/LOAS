import sys
import os
sys.path.append(os.path.join(*['..']*2))
import vpython as vp
from math import *
import numpy as np
from simulator import Simulator
from scao.scao import SCAO
from scao.stabAlgs import PIDRW, PIDMT
from environnement.environment import Environment
from environnement.orbit import Orbit
from hardware.hardwares import Hardware
import matplotlib.pyplot as plt
from scao.quaternion import Quaternion
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
dw = np.array([[0.],[0.],[0.]]) # vecteur de l'accélération angulaire des RW
M = np.array([[0.],[0.],[0.]]) # vecteur du moment magnétique des bobines
I = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite
L0 = np.dot(I,W0)
Wr = []
qs = []

# Environnement
orbite = Orbit(omega, i, e, r_p, mu, tau)
environnement = Environment(B_model)

# Hardware
hardW = Hardware(mgt_parameters, 'custom coil')

# Initialisation du champ magnétique:
orbite.setTime(t)
environnement.setPosition(orbite.getPosition())
B = environnement.getEnvironment()  # dans le référentiel du satellite

# Simulateur
sim = Simulator(dt,L0)

# Algortihmes de stabilisation
stab = SCAO(PIDRW(RW_P,RW_dP,RW_D),PIDMT(MT_P,MT_dP,MT_D),SCAOratio,I,J) #stabilisateur

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

####################
# Fonctions utiles #
####################
def plotAttitude():
    for i in range(4):
        plt.plot([dt*i/orbite.getPeriod() for i in range(len(qs))],[q.vec()[i,0] for q in qs])
    plt.xlabel("t (orbits)")
    plt.ylabel("Quaternion component")
    plt.show()

#####################
# Boucle principale #
#####################
output = {'t':[], 'M':[], 'U':[]}
while True:
    #on récupère la valeur actuelle du champ magnétique et on actualise l'affichage du champ B
    orbite.setTime(t) #orbite.setTime(t)
    environnement.setPosition(orbite.getPosition())
    B = environnement.getEnvironment() #dans le référentiel géocentrique

    # on récupère le prochain vecteur rotation (on fait ube étape dans la sim)
    W = sim.getNextIteration(M,dw,J,B,I)

    # Sauvegarder les valeurs de simulation actuelles:
    stab.setAttitude(sim.Q)
    stab.setRotation(W)
    stab.setMagneticField(B)

    # Enregistrement de variables pour affichage
    Wr.append(np.linalg.norm(W))
    qs.append(sim.Q)

    # Prise de la commande de stabilisation
    dw, M = stab.getCommand(Quaternion(Qt[0][0], Qt[1][0], Qt[2][0], Qt[3][0])) #dans Rv
    U, M = hardW.getRealCommand(dw, M)

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
    output['t'].append(t)
    output['M'].append(M)
    output['U'].append(U)
