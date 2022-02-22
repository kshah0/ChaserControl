function u = controller2(x_chaser_k,x_target_kpN,N,dt_cvx)

% Establish Linear B matrix for state propogation 
B_bar = [1, 0                
         0, cos(x_chaser_k(1))
         0, sin(x_chaser_k(1))];
% Matrix to elimate use of theta in cost function
Q = [0.01,0,0;0,1,0;0,0,1];
P = 3;

relative_heading = atan2((x_target_kpN(3) - x_chaser_k(3)), (x_target_kpN(2) - x_chaser_k(2)));

cvx_begin quiet
    variable u_mat(2,N) 
    variable x_chaser(3,1)
    variable relative_heading
    minimize( norm(Q * (x_target_kpN - x_chaser)) + P*(abs(relative_heading - x_chaser(1))))
    subject to
        x_chaser == x_chaser_k + dt_cvx*B_bar*sum(u_mat')'
        %relative_heading = abs(atan((x_target_kpN(3) - x_chaser(3))/(x_target_kpN(2) - x_chaser(2))) - x_chaser(1))
        -pi/2 <= u_mat(1,:) <= pi/2
        0 <= u_mat(2,:) <= 0.1

cvx_end

u = u_mat(:,1);

end