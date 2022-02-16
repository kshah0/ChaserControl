figure
subplot 211
plot(tspan_sim(1:length(tspan_sim)-1),u_hist(1,:));
ylabel("u_\omega")
title("Control")
subplot 212
plot(tspan_sim(1:length(tspan_sim)-1),u_hist(2,:));
ylabel("u_s")
xlabel("Time (s)");

figure
subplot 311
plot(tspan_sim(1:length(tspan_sim)),rtt,tspan_sim(1:length(tspan_sim)),x_hist(1,:),'LineWidth',2)
ylabel("\theta")
legend("Target","Chaser")
title("Target/Chaser State")
subplot 312
plot(tspan_sim(1:length(tspan_sim)),rtx,tspan_sim(1:length(tspan_sim)),x_hist(2,:),'LineWidth',2)
ylabel("X")
subplot 313
plot(tspan_sim(1:length(tspan_sim)),rty,tspan_sim(1:length(tspan_sim)),x_hist(3,:),'LineWidth',2)
ylabel("Y")
xlabel("Time (s)")


