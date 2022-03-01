function u = controller3(x_chaser_k,x_target_kpN,N,dt_cvx)

% Direction Cosine Matrix
C_k = [cos(x_chaser_k(1)), sin(x_chaser_k(1))
    -sin(x_chaser_k(1)), cos(x_chaser_k(1))];
% Establish Linear B matrix for state propogation 
B_bar = [-C_k(1,2), 0
          C_k(1,1), 0
          0,        C_k(1,1)
          0,        C_k(1,2)];
% Matrix to elimate use of theta in cost function
Q = [0,0,0,0;
     0,0,0,0;
     0,0,1,0;
     0,0,0,1];
P = 3;
%construct DCM state vector
x_chaser_k_dcm = [C_k(1,1);C_k(1,2);x_chaser_k(2);x_chaser_k(3)];
x_target_kpN_dcm = [cos(x_target_kpN(1));sin(x_target_kpN(1));x_target_kpN(2);x_target_kpN(3)];
factor = 1 /  norm(x_target_kpN(2:3) - x_chaser_k(2:3));

cvx_begin
    variable u_mat(2,N) 
    variable x_chaser(4,1)
    variable e
    variable x_star(2,1)
    %variable y_star
    %variable C_star
    minimize( (x_target_kpN_dcm - x_chaser)' * Q * (x_target_kpN_dcm - x_chaser) + P*(e)^2)
    subject to
        x_chaser == x_chaser_k_dcm + dt_cvx*B_bar*sum(u_mat')'
        x_star == (x_target_kpN(2:3) - x_chaser_k_dcm + dt_cvx*B_bar*sum(u_mat')'(3:4)) / norm(x_target_kpN(2:3) - x_chaser_k(2:3))
        %y_star =
        e == -(x_star(2)*C_k(1,1) + x_star(1)*C_k(1,2))
        -pi/2 <= u_mat(1,:) <= pi/2
        0 <= u_mat(2,:) <= 0.1

cvx_end

u = u_mat(:,1);

end