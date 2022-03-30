function u  = controller_nlp_dcm_icost(x_chaser_k,x_target,N,dt)
type objfunx_dcm_fullpos;
u = optimvar('u',2*N);
x_chaser = optimvar('x_chaser',4*(N+1));
x_chaser_k = [cos(x_chaser_k(1));sin(x_chaser_k(1));x_chaser_k(2);x_chaser_k(3)];
% Matrix to elimate use of theta in cost function
Q = [0,0,0,0;
     0,0,0,0;
     0,0,1,0;
     0,0,0,1];
obj = objfunx_dcm_fullpos(x_chaser,Q,x_target);
prob = optimproblem('Objective',obj);

% Constraints
%x_chaser(:,1) = x_chaser_k;
dyn = x_chaser(1:4,1) == x_chaser_k;
for n = 1:N
%     B = [1, 0;                
%          0, cos(x_chaser(1,n));
%          0, sin(x_chaser(1,n))];
    dyn = [dyn; x_chaser(4*n + 1,1) == x_chaser(4*(n-1)+1,1) - dt*x_chaser(4*(n-1)+2)*u(2*(n-1)+1,1)];
    dyn = [dyn; x_chaser(4*n + 2,1) == x_chaser(4*(n-1)+2,1) + dt*x_chaser(4*(n-1)+1)*u(2*(n-1)+1,1)];
    dyn = [dyn; x_chaser(4*n + 3,1) == x_chaser(4*(n-1)+3,1) + dt*x_chaser(4*(n-1)+1)*u(2*(n-1)+2,1)];
    dyn = [dyn; x_chaser(4*n + 4,1) == x_chaser(4*(n-1)+4,1) + dt*x_chaser(4*(n-1)+2)*u(2*(n-1)+2,1)];
end
prob.Constraints.dynamics = dyn;
prob.Constraints.us_top = u(2:2:end,1) <= 0.1;
prob.Constraints.us_bot = u(2:2:end,1) >= 0.0;
prob.Constraints.uw_bot = u(1:2:end,1) >= -pi/2;
prob.Constraints.uw_top = u(1:2:end,1) <= pi/2;

for i = 1:N+1
    x0.x_chaser(4*i-3,1) = x_chaser_k(1);
    x0.x_chaser(4*i-2,1) = x_chaser_k(2);
    x0.x_chaser(4*i-1,1) = x_chaser_k(3);
    x0.x_chaser(4*i,1) = x_chaser_k(4);
end

x0.u = zeros(2*N,1);
%x0.x_chaser = reshape(x_chaser_k.*ones(3,N), [3*(N+1) 1]);

%opts = optimoptions('fmincon', 'Display', 'off');

%show(prob)
[sol,~] = solve(prob,x0);

u = reshape(sol.u,[2,N]);
%u = u(:,1);



end

% function x_chaser_kpN = stateprop(x_chaser_k,u,N)
% x_chaser = x_chaser_k;
% options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
% for i = 1:N-1
%     tspan = i:i+1;
%     [~,x_chaser] = ode45(@dynamics,tspan,x_chaser(:,end),u(:,i),options);
% end
% x_chaser_kpN = x_chaser(:,end);
% end