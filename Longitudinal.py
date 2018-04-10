import numpy as np

m = 6
l = 4

N = np.zeros((l,m))             #veh
maxN = np.ones((l,m)) *20       #veh
Y = np.zeros((l,m+1))           #veh/1dt
maxY = np.ones((l,m+1)) * 1800  #veh/hr
V  = np.zeros((l,m))            #km/hr
Vf = np.ones((l,m)) * 50        #km/hr

L = np.ones(m) * 100    #m
w = 15                  #km/hr
dt = 5                  #sec

Lmin = Vf/3.6 * dt      #m
yin = np.ones((l))      #veh/1dt


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
