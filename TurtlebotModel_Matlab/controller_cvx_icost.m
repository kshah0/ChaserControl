function u_mat = controller_cvx_icost(x_chaser_k,x_target,N,dt)

% Establish Linear B matrix for state propogation 
B_bar = [1, 0                
         0, cos(x_chaser_k(1))
         0, sin(x_chaser_k(1))];
% Matrix to elimate use of theta in cost function
Q = [0.00,0,0;0,1,0;0,0,1];

target_angle_I = atan2(x_target(3,:)-x_chaser_k(3)*ones(1,length(x_target)),x_target(2,:)-x_chaser_k(2)*ones(1,length(x_target)));

cvx_begin quiet
    variable u_mat(2,N)
    variable x_chaser(3,N)
    obj = 0;
    for n = 1:N
        obj = obj + (x_target(:,n) - x_chaser(:,n))'*Q*(x_target(:,n) - x_chaser(:,n));
        obj = obj + (target_angle_I(n)-x_chaser(1,n))'*2*(target_angle_I(n)-x_chaser(1,n));
    end
    minimize(obj)
    %minimize(sum(((x_target - x_chaser)'*Q*(x_target - x_chaser))'))
    subject to
        x_chaser(:,1) == x_chaser_k;
        for n = 2:N
            x_chaser(:,n) == x_chaser(:,n-1) + dt*B_bar*u_mat(:,n-1);
        end

        -pi/2 <= u_mat(1,:) <= pi/2
        0 <= u_mat(2,:) <= 0.1
        u_mat(1,:).^2+u_mat(2,:).^2 <= .695
    
cvx_end


       