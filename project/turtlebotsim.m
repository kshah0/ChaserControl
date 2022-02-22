clear;clc;
%% Configuration
%Initial Chaser State
x0 = [pi/2;0;0.2];
u0 = [0;0];
%Target Trajectory
rty = [1:.2/400:1.2,1.2*ones(1,100+400+99),1.2:.2/600:1.4]'-.9*ones(1601,1);
rtx = [ones(1,400+100),1.0:.2/400:1.2,1.2*ones(1,600+100)]'-.9*ones(1601,1);
rtt = [pi/2*ones(1,400), pi/2:-pi/200:0, zeros(1,399), 0:pi/200:pi/2, pi/2*ones(1,600)]';
refTraj = [rtt rtx rty];

%Sim/Controller Rates
dt_sim = 0.01;
dt_controller = 0.5;
tf = 16;

%MPC configuration
dt_cvx = .01;
N = 100; % MPC predicition horizon

%% Generate & Run Simulation
ovsf = dt_controller/dt_sim; %oversampling factor
tspan_sim = 0:dt_sim:tf;
tspan_controller = 0:dt_controller:tf;

options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);

xk = x0;
u_hist = []; %make global probs
x_hist = x0;
for k = 1:length(tspan_controller)-1
    tspan_ode = tspan_controller(k):dt_sim:tspan_controller(k+1);
    if k*ovsf + N*dt_cvx/dt_sim <= length(refTraj(:,1))
        x_target_kpn = interp1(tspan_sim,refTraj,tspan_sim(k*ovsf + N*dt_cvx/dt_sim))';
    else
        x_target_kpn = interp1(tspan_sim,refTraj,tspan_sim(end))';
    end
    u = controller(xk,x_target_kpn,N,dt_cvx);
    [~,x] = ode45(@dynamics,tspan_ode,xk,options,u);
    x_hist = [x_hist, x(1:end-1,:)'];
    u_hist = [u_hist, u.*ones(2,ovsf)];
    xk = x(end,:)';
    disp(string(x_target_kpn(2) - xk(2)) + ", " + string(x_target_kpn(3) - xk(3)))
    %disp(xk(2) + ", " + xk(3))
    if abs(x_target_kpn(2) - xk(2)) < 0.01 && abs(x_target_kpn(3) - xk(3)) < 0.01
        disp("Target Hit!")
        break
    end
end




% Plot graphs
plot_trajectory;
plot_data;
