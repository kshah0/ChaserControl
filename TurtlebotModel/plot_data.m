% Control Plot
figure
subplot 211
plot(tspan_sim(1:length(u_hist(1,:))),u_hist(1,:),"LineWidth",2);
ylabel("u_\omega")
title("Control")
subplot 212
plot(tspan_sim(1:length(u_hist(1,:))),u_hist(2,:),"LineWidth",2);
ylabel("u_s")
xlabel("Time (s)");

% Target vs. Chaser State Plot
figure
subplot 311
plot(tspan_sim(1:length(x_hist(1,:))),rtt(1:length(x_hist(1,:))),tspan_sim(1:length(x_hist(1,:))),x_hist(1,:),'LineWidth',2)
ylabel("\theta")
legend("Target","Chaser")
title("Target/Chaser State")
subplot 312
plot(tspan_sim(1:length(x_hist(1,:))),rtx(1:length(x_hist(1,:))),tspan_sim(1:length(x_hist(1,:))),x_hist(2,:),'LineWidth',2)
ylabel("X")
subplot 313
plot(tspan_sim(1:length(x_hist(1,:))),rty(1:length(x_hist(1,:))),tspan_sim(1:length(x_hist(1,:))),x_hist(3,:),'LineWidth',2)
ylabel("Y")
xlabel("Time (s)")


