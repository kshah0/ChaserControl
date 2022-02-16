close all

figure(1)
hold on;
scatter(rtx,rty)
scatter(x_hist(2,:),x_hist(3,:));
legend("Target","Chaser")
title("Ground Trajectory")
xlabel("X")
ylabel("Y")
axis equal