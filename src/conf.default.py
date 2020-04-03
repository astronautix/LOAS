import numpy as np
from random import *
from math import *
###############################
# Paramètres de la simulation #
###############################
# Temps
dt = 1/25 #pas de temps de la simulation
fAffichage = 25 #fréquence d'affichage

# Géométrie
lx,ly,lz = 10,10,10 #longueur du satellit selon les axes x,y,z
m = 1 #masse du satellite
I0 = np.diag((m*(ly**2+lz**2)/3,m*(lx**2+lz**2)/3,m*(lx**2+ly**2)/3)) # Tenseur inertie du satellite

# Mouvement
W0 = 0*np.array([[2*(random()-0.5)] for i in range(3)]) #rotation initiale dans le référentiel R_r
J = 1
