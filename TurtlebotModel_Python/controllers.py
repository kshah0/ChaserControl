# from turtle import pd
import numpy as np
import cvxpy as cvx

def cvx_controller(x_chaser_k, x_target, N, dt):
    # N is the number of controller commands, there are N+1 states from integration
    B_bar = np.array([[1.0, 0.0],
                      [0.0, np.cos(x_chaser_k[0])],
                      [0.0, np.sin(x_chaser_k[0])]])
    Q = np.diag([0.0, 1.0, 1.0])
    R = 2
    
    uw_max = np.pi/2
    us_max = .15
    
    ref_angle_I = np.arctan2(x_target[2] - x_chaser_k[2]*np.ones((1,len(x_target[0]))),x_target[1] - x_chaser_k[1]*np.ones((1,len(x_target[0]))))
    u_mat = cvx.Variable((2,N))
    x_chaser = cvx.Variable((3,N+1))
    J = 0
    for n in range(N):
        # import pdb;pdb.set_trace()
        # J += (x_target[:,n] - x_chaser[n])@Q@(x_target[:,n] - x_chaser[n])
        J += cvx.sum_squares(Q@(x_target[:,n]-x_chaser[:,n]))
        
        J += R*(ref_angle_I[0,n] - x_chaser[0,n])**2
        
    obj = cvx.Minimize(J)
    
    constr = [x_chaser[:,0] == x_chaser_k]
    
    
    for t in range(N):
        # import pdb;pdb.set_trace()
        constr += [x_chaser[:,t+1] == x_chaser[:,t] + dt*B_bar@u_mat[:,t],
                  u_mat[0,t]<=uw_max, u_mat[0,t]>=-uw_max,
                  u_mat[1,t]<=us_max, u_mat[1,t]>=0,
                  (u_mat[0,t]/uw_max)**2 + (u_mat[1,t]/us_max)**2 <= np.sqrt(2)]
    
    prob = cvx.Problem(obj, constr)
    result = prob.solve()
    # print(J.value)
    # import pdb;pdb.set_trace()
    return u_mat.value.T

def cvx_controller_predict(x_chaser_k, x_target, N, dt):
    
    x_target_k = x_target[:,0]
    wt = (x_target[0,1]-x_target[0,0])/dt
    xdott = (x_target[1,1]-x_target[1,0])/dt
    ydott = (x_target[2,1]-x_target[2,0])/dt
    tVect = np.arange(0,(N+1)*dt,dt)
    #import pdb;pdb.set_trace()
    x_target = np.array([wt*tVect+x_target_k[0]*np.ones((1,len(tVect)))[0],xdott*tVect+x_target_k[1]*np.ones((1,len(tVect)))[0],ydott*tVect+x_target_k[2]*np.ones((1,len(tVect)))[0]])
    
    # N is the number of controller commands, there are N+1 states from integration
    B_bar = np.array([[1.0, 0.0],
                      [0.0, np.cos(x_chaser_k[0])],
                      [0.0, np.sin(x_chaser_k[0])]])
    Q = np.diag([0.0, 1.0, 1.0])
    R = 2
    
    uw_max = np.pi/2
    us_max = .15
    
    ref_angle_I = np.arctan2(x_target[2] - x_chaser_k[2]*np.ones((1,len(x_target[0]))),x_target[1] - x_chaser_k[1]*np.ones((1,len(x_target[0]))))
    u_mat = cvx.Variable((2,N))
    x_chaser = cvx.Variable((3,N+1))
    J = 0
    for n in range(N):
        # import pdb;pdb.set_trace()
        # J += (x_target[:,n] - x_chaser[n])@Q@(x_target[:,n] - x_chaser[n])
        J += cvx.sum_squares(Q@(x_target[:,n]-x_chaser[:,n]))
        
        J += R*(ref_angle_I[0,n] - x_chaser[0,n])**2
        
    obj = cvx.Minimize(J)
    
    constr = [x_chaser[:,0] == x_chaser_k]
    
    
    for t in range(N):
        # import pdb;pdb.set_trace()
        constr += [x_chaser[:,t+1] == x_chaser[:,t] + dt*B_bar@u_mat[:,t],
                  u_mat[0,t]<=uw_max, u_mat[0,t]>=-uw_max,
                  u_mat[1,t]<=us_max, u_mat[1,t]>=0,
                  (u_mat[0,t]/uw_max)**2 + (u_mat[1,t]/us_max)**2 <= np.sqrt(2)]
    
    prob = cvx.Problem(obj, constr)
    result = prob.solve()
    # print(J.value)
    # import pdb;pdb.set_trace()
    return u_mat.value.T