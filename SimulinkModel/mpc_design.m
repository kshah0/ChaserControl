function u_output = mpc_design(x_input)
%x_input = [t, N, x_chaser_k, x_target_kpN, uw_mat, us_mat];
t = x_input(1);
N = x_input(2);
x_chaser_k = x_input(3:5);
x_target_kpN = x_input(6:8);
uw_mat = x_input(9:9+N);
us_mat = x_input((9+N):(9+2*N));
T_opt = .5;
dt = 0.01;
if mod(t,T_opt) == 0

% Establish Linear B matrix for state propogation 
B_bar = [1, 0                
         0, cos(x_chaser_k(1))
         0, sin(x_chaser_k(1))];
% Matrix to elimate use of theta in cost function
Q = [1,0,0;0,1,0;0,0,1];

%x_chaser_kpN = [-inf;-inf;-inf];

cvx_begin
    variable u_mat(2,N) 
    variable x_chaser(3,1)
    minimize( norm(Q * (x_target_kpN - x_chaser)))
    subject to
        x_chaser == x_chaser_k + dt*B_bar*sum(u_mat')'
        -pi/2 <= u(1,:) <= pi/2
        0 <= u(2,:) <= 0.1

cvx_end

u_output = [u_mat(:,1)',u_mat(1,:),u_mat(2,:)]; % [current control cmd, future uw cmds, future us cmds]
else
    u_output = [u_mat(:,int(mod(t,T_opt)/dt))',uw_mat,us_mat];
end
end