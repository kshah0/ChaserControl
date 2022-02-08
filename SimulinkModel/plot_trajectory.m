close all
data = load("simresults/run1.mat");

figure(1)
hold on;
scatter(rtx,rty)
scatter(data.run{2}.Values.Data,data.run{3}.Values.Data)
legend("Target","Chaser")
title("Ground Trajectory")
xlabel("X")
ylabel("Y")