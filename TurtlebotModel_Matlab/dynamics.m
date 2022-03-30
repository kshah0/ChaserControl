function xdot = dynamics(t,x,u)
% STATE VECTOR X: [theta,x,y], [orientation, x pos inertial, y pos inertial
% (pose state)
% Pull the future position of target from table lookup


% CONTROL INPUT u: [uw,uv], [angular rate, linear velocity] (twist control)
xdot = zeros(3,1);

xdot(1) = u(1); % theta_dot
xdot(2) = u(2)*cos(x(1)); %x inertial velocity
xdot(3) = u(2)*sin(x(1)); %y inertial velocity

end