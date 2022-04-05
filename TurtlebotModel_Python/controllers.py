import numpy as np
import cvxpy as cvx

def cvx_controller(x_chaser_k, x_target, N, dt):
    B_bar = np.array([[1, 0],
                      [0, np.cos(x_chaser_k[0])],
                      [0, np.sin(x_chaser_k[0])]])
    Q = np.diag([0, 1, 1])
    R = 2
    
    uw_max = np.pi/2
    us_max = .15
    
    ref_angle_I = np.arctan2(x_target[2] - x_chaser_k[2]*np.ones((1,len(x_target[0]))),x_target[1] - x_chaser_k[1]*np.ones((1,len(x_target[0]))))
    u_mat = cvx.Variable((N-1,2))
    x_chaser = cvx.Variable((N,3))
    J = 0
    for n in range(N):
        # import pdb;pdb.set_trace()
        # J += (x_target[:,n] - x_chaser[n])@Q@(x_target[:,n] - x_chaser[n])
        J += Q[1,1]*(x_target[1,n]-x_chaser[n,1])**2 + Q[2,2]*(x_target[2,n]-x_chaser[n,2])**2
        
        J += R*(ref_angle_I[0,n] - x_chaser[n,0])**2
        
    obj = cvx.Minimize(J)
    
    constr = []
    x_chaser[0,:] == x_chaser_k
    
    for t in range(N-1):
        # import pdb;pdb.set_trace()
        constr += [x_chaser[t+1] == x_chaser[t] + dt*B_bar@u_mat[t],
                  u_mat[t,0]<=uw_max, u_mat[t,0]>=-uw_max,
                  u_mat[t,1]<=us_max, u_mat[t,1]>=0]#,
                #   (u_mat[t,0]/us_max)**2 + (u_mat[t,1]/uw_max)**2 <= .707]
    
    prob = cvx.Problem(obj, constr)
    print(prob)
    result = prob.solve()
    return u_mat.value

xt = np.array([[0,0,0,0],[0,0,0,0],[0.0,0.05,0.1,0.15]])
a = cvx_controller(np.array([0,0,0]), xt, 4, 1)
print(a)
    #https://colab.research.google.com/github/cvxgrp/cvx_short_course/blob/master/intro/control.ipynb