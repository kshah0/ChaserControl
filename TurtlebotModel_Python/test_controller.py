import numpy as np
def test_controller(xk, x_target,N,dt_ctrl):
    return np.array([0.05,0.1])*np.ones((len(x_target[0]),2))