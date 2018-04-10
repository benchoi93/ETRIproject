import numpy as np

m = 6
l = 4

N = np.zeros((l,m))
maxN = np.ones((l,m)) *20 #veh
Y = np.zeros((l,m+1))
maxY = np.ones((l,m+1)) * 1800 #veh/hr
V  = np.zeros((l,m))
Vf = np.ones((l,m)) * 50

L = np.ones(m) * 100
w = 15
dt = 5

Lmin = Vf/3.6 * dt
yin = np.ones((l))


Y[:,0]  = yin
Y[:,1:] = np.minimum(np.minimum(Lmin/L*N    , maxY[:,:-1]) , 
                     np.minimum(maxY[:,1:]  , w*dt/L *(maxN - N) ) 
                     )
# vehicle list scan --> change moveForward flag
N += Y[:,:-1]


