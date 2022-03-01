function u = controller5(x_chaser_k,x_target_kpN,N,dt_cvx)

% Establish Linear B matrix for state propogation 
B_bar = [1, 0                
         0, cos(x_chaser_k(1))
         0, sin(x_chaser_k(1))];
% Matrix to elimate use of theta in cost function
Q = [0.05,0,0;0,100,0;0,0,100];


cvx_begin quiet
    variable u_mat(2,N) 
    variable x_chaser(3,N)
    minimize( norm(Q * (x_target_kpN - x_chaser(1:3,N))))
    subject to
        x_chaser(:,1) == x_chaser_k
        for n = 1:N-1
            B = [1, 0;                
                 0, cos(x_chaser(1,n));
                 0, sin(x_chaser(1,n))]
            x_chaser(1:3,n+1) == x_chaser(1:3,n) + dt_cvx*B*u_mat(1:2,n)
        end
        -pi/2 <= u_mat(1,:) <= pi/2
        0 <= u_mat(2,:) <= 0.1

cvx_end

u = u_mat(:,1);

end