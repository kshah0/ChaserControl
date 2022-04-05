import numpy as np
def dynamics(x,t,uk):
    xdot = np.array([0.0,0.0,0.0])
    
    xdot[0] = uk[0]
    xdot[1] = uk[1]*np.cos(x[0])
    xdot[2] = uk[1]*np.sin(x[0])
    
    return xdot