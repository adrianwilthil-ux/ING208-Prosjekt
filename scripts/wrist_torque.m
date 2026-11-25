addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

qa = qz + [0, -pi/4, 0, 0, pi/4 - pi];
qb = qa + [0, 0, 0, pi, 0];
qc = qa + [0, 0, 0, 0, pi];

dt = 0.1;
Tb = 4; 
Nb = round(Tb / dt) + 1; 
tb = (0:Nb-1)' * dt; % Time vector
sb = tpoly(0, 1, tb); % Interpolation variable

[q1, qd1, qdd1] = jtraj(qa, qb, sb);
[q2, qd2, qdd2] = jtraj(qa, qc, sb);

close all
robot.plot(q1);
robot.plot(q2);

trq1 = robot.rne(q1, qd1, qdd1);
trq2 = robot.rne(q2, qd2, qdd2);

plot_dynamics(trq1, dt, q1, qd1, qdd1);
plot_dynamics(trq2, dt, q2, qd2, qdd2);

disp("Peak w1 moment [Nm]:")
disp(max(trq1(:, 4)));
disp("Peak w2 moment [Nm];")
disp(max(trq2(:, 5)));