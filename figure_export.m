% Extract signals
t = out.ey_out.Time;

ey = out.ey_out.Data;
ephi = out.ephi_out.Data;
delta = out.delta_out.Data;
u = out.u_out.Data;

% Create performance dashboard
figure('Name','LKA Performance Evaluation');

subplot(2,2,1)
plot(t,ey,'LineWidth',1.5);
grid on;
title('Lateral Position Error');
xlabel('Time (s)');
ylabel('e_y (m)');

subplot(2,2,2)
plot(t,ephi,'LineWidth',1.5);
grid on;
title('Heading Angle Error');
xlabel('Time (s)');
ylabel('e_\phi (rad)');

subplot(2,2,3)
plot(t,delta,'LineWidth',1.5);
hold on;
yline(0.5,'--r');
yline(-0.5,'--r');
grid on;
title('Steering Angle');
xlabel('Time (s)');
ylabel('\delta (rad)');

subplot(2,2,4)
plot(t,u,'LineWidth',1.5);
grid on;
title('Controller Command');
xlabel('Time (s)');
ylabel('u');

sgtitle('Lane Keeping Assist - Performance Evaluation');