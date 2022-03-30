%Target Trajectory
rty = [1:.2/400:1.2,1.2*ones(1,100+400+99),1.2:.2/600:1.4]'-.9*ones(1601,1);
rtx = [ones(1,400+100),1.0:.2/400:1.2,1.2*ones(1,600+100)]'-.9*ones(1601,1);
rtt = [pi/2*ones(1,400), pi/2:-pi/200:0, zeros(1,399), 0:pi/200:pi/2, pi/2*ones(1,600)]';
dt = 0.01;
tspan_target = 0:dt:16;

refTraj = [rtt rtx rty];