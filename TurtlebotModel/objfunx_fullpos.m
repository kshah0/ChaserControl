function J = objfunx_fullpos(x_chaser,Q, x_target)

%x_chaser = reshape(x_chaser,[3,length(x_chaser/3)]);

J = 0;
for k = 1:length(x_target)
    J = J + (x_target(:,k) - [x_chaser(3*k-2);x_chaser(3*k-1);x_chaser(3*k)])' * Q * (x_target(:,k) - [x_chaser(3*k-2);x_chaser(3*k-1);x_chaser(3*k)]);
end
end