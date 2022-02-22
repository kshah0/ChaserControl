figure
subplot 211
%plot(tspan_sim(1:length(tspan_sim)-1),u_hist(1,:));
plot(tspan_sim(1:k*ovsf),u_hist(1,:));
ylabel("u_\omega")
title("Control")
subplot 212
%plot(tspan_sim(1:length(tspan_sim)-1),u_hist(2,:));
plot(tspan_sim(1:k*ovsf),u_hist(2,:));
ylabel("u_s")
xlabel("Time (s)");

figure
subplot 311
plot(tspan_sim(1:k*ovsf+1),rtt(1:k*ovsf+1),tspan_sim(1:k*ovsf+1),x_hist(1,:),'LineWidth',2)
ylabel("\theta")
legend("Target","Chaser")
title("Target/Chaser State")
subplot 312
plot(tspan_sim(1:k*ovsf+1),rtx(1:k*ovsf+1),tspan_sim(1:k*ovsf+1),x_hist(2,:),'LineWidth',2)
ylabel("X")
subplot 313
plot(tspan_sim(1:k*ovsf+1),rty(1:k*ovsf+1),tspan_sim(1:k*ovsf+1),x_hist(3,:),'LineWidth',2)
ylabel("Y")
xlabel("Time (s)")


