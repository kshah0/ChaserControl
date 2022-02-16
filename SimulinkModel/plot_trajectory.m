close all
data = load("simresults/makingprogress.mat");

figure(1)
hold on;
scatter(rtx,rty)
scatter(reshape(data.run{4}.Values.Data,[length(data.run{4}.Values.Data) 1]),reshape(data.run{5}.Values.Data,[length(data.run{4}.Values.Data) 1]))
legend("Target","Chaser")
title("Ground Trajectory")
xlabel("X")
ylabel("Y")