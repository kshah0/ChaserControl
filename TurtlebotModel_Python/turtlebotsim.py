import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import odeint
from dynamics import dynamics
from test_controller import test_controller
from controllers import cvx_controller
import matplotlib.pyplot as plt

#Chaser initial state
x0c = np.array([np.pi,0,0])
u0c = np.array([0,0])

#Define reference trajectory
tf = 16
x0t = np.array([np.pi,0.1,0.1])
t_ref = np.array([0,5,6,11,12,tf])
tt = np.array([x0t[0], x0t[0], x0t[0]-np.pi,x0t[0]-np.pi,x0t[0],x0t[0]])
xt = np.array([x0t[1], x0t[1], x0t[1],x0t[1]+.25,x0t[1]+.25,x0t[1]+.25])
yt = np.array([x0t[2], x0t[2]+.25, x0t[2]+.25,x0t[2]+.25,x0t[2]+.25,x0t[2]+.4])
rtraj = np.array([tt,xt,yt]).T

target_state = interp1d(t_ref, rtraj.T)

#Define all time scales
#Time step for the simulation
dt_sim = 0.01
#Time duration for ZOH controller commands
dt_ctrl = .2
#Time duration between reoptimizing
dt_opt = 1

ovsf = int(dt_ctrl/dt_sim)

N = 10
if(N*dt_ctrl < dt_opt):
    print("Error: control horiziation insufficient for optimizer")
    quit()

#Generate timespan arrays
tspan_sim = np.arange(0,tf,dt_sim)
tspan_ctrl = np.arange(0,dt_opt,dt_ctrl)
tspan_opt = np.arange(0,tf,dt_opt)
tspan_ode = np.arange(0,dt_ctrl, dt_sim)

xk = x0c
x_hist = np.array([x0c])
u_hist = np.array([u0c])
for k in range(len(tspan_opt)-1):
    if(tspan_opt[k] + N*dt_ctrl < tf):
        x_target = target_state(np.arange(tspan_opt[k],tspan_opt[k] + N*dt_ctrl,dt_ctrl))
    else:
        x_target = (target_state(tspan_opt[-1])*np.ones((N,3))).T
    
    uk = cvx_controller(xk, x_target,N,dt_ctrl)
    
    xc = xk
    for c in range(len(tspan_ctrl)-1):
        #ODE45 call here
        x = odeint(dynamics,xc,tspan_ode,args = (uk[c],))
        x_hist = np.concatenate((x_hist,x[0:-1]))
        u_hist = np.concatenate((u_hist,uk[c]*np.ones((ovsf,2))))
        xc = x[-1]
        #Append xhist and u hist
        #xc = x(end,:)
    xk = xc
#append final state vector to x_hist
np.concatenate((x_hist,np.array([xk])))
#import pdb; pdb.set_trace()
#plot stuff
plt.figure(figsize=[8,5])
plt.scatter(x_hist.T[:][1],x_hist.T[:][2])
plt.scatter(target_state(tspan_sim)[:][1],target_state(tspan_sim)[:][2])
plt.axis('equal')
plt.show()