clear;clc;
x0 = [0;0;0];
u0 = [0;0];
rty = [1:.2/400:1.2,1.2*ones(1,100+400+99),1.2:.2/600:1.4]'-.9*ones(1601,1);
rtx = [ones(1,400+100),1.0:.2/400:1.2,1.2*ones(1,600+100)]'-.9*ones(1601,1);
rtt = [zeros(1,400), 0:pi/200:pi/2, pi/2*ones(1,399), pi/2:-pi/200:0, zeros(1,600)]';
refTraj = [rtt rtx rty];
N = 50; % MPC predicition horizon
