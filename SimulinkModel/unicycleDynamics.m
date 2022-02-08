function xdot = unicycleDynamics(x,u,Ts)
xdot(1,1) = u(1);
xdot(2,1) = u(2)*sin(x(1));
xdot(3,1) = u(2)*cos(x(1));
end