import numpy as np

m = 6 # number of cells in current link
l = 4 # maximum number of lanes in current link

N = np.zeros((l,m))             # number of vehicles [veh]
maxN = np.ones((l,m)) *20       # Max. number of vehicles [veh]
Y = np.zeros((l,m+1))           # Outflow [veh/1dt]
maxY = np.ones((l,m+1)) * 1800  # capacity flow [veh/hr]
V  = np.zeros((l,m))            # Cell average speed [km/hr]
Vf = np.ones((l,m)) * 50        # Cell free flow speed [km/hr]

L = np.ones(m) * 100    # Length of the Cells [m]
w = 15                  # wave speed [km/hr]
dt = 5                  # simulation time step [sec]

Lmin = Vf/3.6 * dt      # minimum cell length (CTM assumption) [m]
yin = np.ones((l))      # Link inflow [veh/1dt]


Y[:,0]  = yin
Y[:,1:] = np.minimum(np.minimum(Lmin/L*N    , maxY[:,:-1]) , 
                     np.minimum(maxY[:,1:]  , w*dt/L *(maxN - N) ) 
                     )
"""
vehicle list scan 
  --> change moveForward flag
"""

N += Y[:,:-1]

"""
vehicle list scan 
  --> change currentCell
  --> change currentLane
"""
