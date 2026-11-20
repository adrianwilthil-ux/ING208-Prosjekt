function plot_dynamics(torque, dt, q, qd, qdd)

n = length(torque);
t = (0:n-1)' * dt;


figure;
subplot(3,1,1)
plot(t, q)
ylabel('Position (rad)')
legend('Shoulder', 'Elbow')
grid on

subplot(3,1,2)
plot(t, qd)
ylabel('Velocity (rad/s)')
grid on

subplot(3,1,3)
plot(t, qdd)
ylabel('Acceleration (rad/s^2)')
xlabel('Time (s)')
grid on

figure;
plot(t, torque)
xlabel('Time (s)')
ylabel('Joint torque (N m)')
legend('Base', 'Shoulder', 'Elbow', 'W1', 'W2')
grid on