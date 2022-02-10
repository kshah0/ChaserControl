function u = mpc_design(x_chaser_k,x_target_kpN)
N = 10;
% Establish Linear B matrix for state propogation 
B_bar = [0, cos(x_chaser_k(1))
         0, sin(x_chaser_k(1))
         1, 0                ];
% Matrix to elimate use of theta in cost function
Q = [0,0,0;0,1,0;0,0,1];
dt = 0.01;
x_chaser_kpN = x_chaser_k;

cvx_begin
    variable u(2,N)
    minimize( norm( Q *(x_target_kpN - x_chaser_kpN)))
    subject to
        x_chaser_kpN == x_chaser_k + dt*B_bar*sum(u')'
        u(1,:) <= 0.1
        u(1,:) >= 0
        u(2,:) <= pi/2
        u(2,:) >= -pi/2
cvx_end


end