import numpy as np
def dynamics(t,x,u):
    xdot = np.zeros((3,1))
    
    xdot[0] = u[0]
    xdot[1] = u[1]*np.cos(x[0])
    xdot[2] = u[1]*np.sin(x[0])
    
    return xdot