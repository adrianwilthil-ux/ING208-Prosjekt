addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

a = [1, 3; 2, 5; 4, 1; 2, 1; -2, -3; -4, 3];
t = linspace(0, 3, length(a));

[~, da] = gradient(a);

plot(t, a)
plot(t, da)



q1 = [0, 0, 0, 0, 0];
q2 = [1.5708    0.2429    0.4047         0   -0.6477];
t = linspace(0, 3, 100);

[q, qd, qdd] = jtraj(q1, q2, t);

%robot.plot(q);
torque = robot.rne(q, qd, qdd);

% figure;
% subplot(3,1,1)
% plot(t, q)
% ylabel('Position (rad)')
% legend('Shoulder', 'Elbow')
% grid on
% 
% subplot(3,1,2)
% plot(t, qd)
% ylabel('Velocity (rad/s)')
% grid on
% 
% subplot(3,1,3)
% plot(t, qdd)
% ylabel('Acceleration (rad/s^2)')
% xlabel('Time (s)')
% grid on
% 
% figure;
% plot(t, torque)
% xlabel('Time (s)')
% ylabel('Joint torque (N m)')
% legend('Base', 'Shoulder', 'Elbow', 'W1', 'W2')
% grid on


