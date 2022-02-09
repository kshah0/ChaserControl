clear;clc;
x0 = [0;0;0];
u0 = [0;0];
rty = [1:.2/400:1.2,1.2*ones(1,100+400+99),1.2:.2/600:1.4]'-.5*ones(1601,1);
rtx = [ones(1,400+100),1.0:.2/400:1.2,1.2*ones(1,600+100)]'-.5*ones(1601,1);
rtt = [zeros(1,400), 0:pi/200:pi/2, pi/2*ones(1,399), pi/2:-pi/200:0, zeros(1,600)]';
refTraj = [rtt rtx rty];
t = [0:.01:16];

%% Model Formulation for MPC
controller = nlmpc(3,3,2); % npy 2 or 3?
controller.Model.StateFcn = "unicycleDynamics";
controller.Model.OutputFcn = @(x,u,Ts) [x(1); x(2); x(3)];
controller.Ts = 1;
controller.PredictionHorizon = 5;
controller.Model.NumberOfParameters = 0;
% controller.Optimization.RunAsLinearMPC = 'off';
% controller.Optimization.ReplaceStandardCost = false;
% Control Constraints
% controller.ManipulatedVariables(1).Name = "uw"; 
% controller.ManipulatedVariables(1).Min = -pi/2;
% controller.ManipulatedVariables(1).Max = pi/2;
% controller.ManipulatedVariables(1).RateMin= -.25;
% controller.ManipulatedVariables(1).RateMax= .25;
% controller.ManipulatedVariables(2).Name = "uv";
% controller.ManipulatedVariables(2).Min = 0;
% controller.ManipulatedVariables(2).Max = .1;
% controller.ManipulatedVariables(2).RateMin= -.2;
% controller.ManipulatedVariables(2).RateMax= .2;
% 
% controller.Weights.ManipulatedVariables = [1 1];
% validateFcns(controller,x0,u0);

