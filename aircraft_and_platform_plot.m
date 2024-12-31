%close all; clear;
% % Define simulation time
% sim_time = 30;  % seconds
% 
% % Run the simulation
% sim('AircraftPlatformModel', sim_time);

% Plot ideal position signal and position tracking
close all;
figure(1);
%subplot(211);
% plot(t, x(:,1), 'r', t, x(:,2), 'b', 'linewidth', 2);  % Ideal and tracked position
plot(t, 500 * exp(-0.2839 * t), 'r', t, x(:,3), 'b', 'linewidth', 2);  % Ideal and tracked position

legend('Desired aircraft position', 'Ground platform position tracking with SMC');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;
% Plot ideal speed signal and speed tracking
figure(2);
%subplot(212);
% plot(t, 70*cos(t), 'r', t, x(:,3), 'b', 'linewidth', 2);  % Ideal speed (cos(t)) and tracked speed
plot(t, (0.2839) * 250 * exp(-0.2839 * t), 'r', t, x(:,2), 'b', 'linewidth', 2);  % Ideal and tracked position
legend('Desired aircraft velocity', 'Ground platform velocity tracking with SMC');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;
% Plot control input (ut)
figure('Name','Control effort with SMC','NumberTitle','off','Position',[100, 100, 800, 600]);
%figure(3);
plot(t, ut(:,1)/10^4, 'r', 'linewidth', 2);  % Control input over time
title('Control effort with SMC');
xlabel('Time (s)');
ylabel('Control effort (m^2/s)');
grid on;
% % Plot position & velocity
% figure(4);
% plot(t, x(:,1), 'r', t, x(:,2), 'b', 'linewidth', 2);  % Ideal and tracked position
% legend('desired aircraft position', 'ground platform Position tracking');
% xlabel('time (s)');
% ylabel('Position (m)');