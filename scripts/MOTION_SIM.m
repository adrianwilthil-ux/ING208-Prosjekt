%% MOTION SIM TOTAL - Zero-position -> Drills n holes -> Zero-position
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;
close all %Clearing previous figures

%% Input

% Points on ceiling
p1_ceil = [0, -0.5]; % [m]
p2_ceil = [0, 0.5]; % [m]
p3_ceil = [0, 0]; % [m]

% Tilt
tilt_mag = 15; % Tilt magnitude [deg]
tilt_phi = 45; % Tilt direction [deg]


points = [p1_ceil; p2_ceil; p3_ceil];
P0 = double(robot.fkine(qz)); % Cartesian 0-pose


%% Time and interpolation
dt = 0.1; % Simulation time step

%Between holes
Tb = BETWEEN_HOLES_TIME; % Between holes motion time
Nb = round(Tb / dt) + 1; % Amount of samples (for between motion)
tb = (0:Nb-1)' * dt; % Time vector
sb = tpoly(0, 1, tb); % Interpolation variable

%Drill approach
Ta = APPROACH_TIME; % Approach time
Na = round(Ta / dt) + 1; % Amount of samples (for approach motion)
ta = (0:Na-1)' * dt; % Time vector
d = tpoly(0, CLEARANCE, ta, 0, DRILL_SPEED_UP); 
sa = d / CLEARANCE; % Interpolation variable

%Drilling motion

T = ceil((DRILL_STROKE/DRILL_SPEED_UP + 0.5/2) / dt) * dt; % Drilling motion time
t = (0:round(T/dt))' * dt; % Time vector

t_brake = 2*(T - DRILL_STROKE/DRILL_SPEED_UP); % Slowing duration
u = max(0, t - (T - t_brake)); % Time elapsed since braking began
d = DRILL_SPEED_UP*t - DRILL_SPEED_UP*u.^2/(2*t_brake);% Distance along drill axis [m]
sd = d / DRILL_STROKE; %Interpolation variable

%Drill retract
Tr = RETRACT_TIME; % Retract time
Nr = round(Tr / dt) + 1; % Amount of samples
tr = (0:Nr-1)' * dt; % Time vector
sr = tpoly(0, 1, tr); % Interpolation variable





%% Main

% Plotting disc
plot_disc(points, CEILING, tilt_mag, tilt_phi);

% Initializing position, velocity, acceleration and torque variables
full_q = zeros(0, 5);
full_qd = zeros(0, 5);
full_qdd = zeros(0, 5);
full_trq = zeros(0, 5);

% Plot robot in 0-pose
robot.plot(qz, 'view', [45 30]);

% Main loop, loops over every hole
for i = 1:size(points, 1)
    P = ceiling_to_base(points(i, 1), points(i, 2), CEILING, CLEARANCE, tilt_mag, tilt_phi);  % Clearance pose
    A = ceiling_to_base(points(i, 1), points(i, 2), CEILING, 0, tilt_mag, tilt_phi);  % Approach pose
    DRILL_STROKE = ceiling_to_base(points(i, 1), points(i, 2), CEILING, -DRILL_STROKE, tilt_mag, tilt_phi);  % Final drill pose
    
    % Get clearance point values and plot
    [q_between, qd_between, qdd_between, trq_between] = between_traj2(P, robot, sb, tb, dt);
    robot.plot(q_between);
    
    % Get drill motion values and plot
    [q_drill, qd_drill, qdd_drill, trq_drill] = drill_traj(P, A, DRILL_STROKE, robot, sd, sa, sr, q_between(end, :), dt, DRILL_FORCE, DRILL_MOMENT);
    robot.plot(q_drill);

    % Store values
    full_q = [full_q; q_between; q_drill];
    full_qd = [full_qd; qd_between; qd_drill];
    full_qdd = [full_qdd; qdd_between; qdd_drill];
    full_trq = [full_trq; trq_between; trq_drill];

end

% Reset to 0-pose and plot
[q_reset, qd_reset, qdd_reset] = between_traj2(double(robot.fkine(qz)), robot, sb, tb, dt);
trq_reset = robot.rne(q_reset, qd_reset, qdd_reset);
robot.plot(q_reset);

% Store
full_q = [full_q; q_reset];
full_qd = [full_qd; qd_reset];
full_qdd = [full_qdd; qdd_reset];
full_trq = [full_trq; trq_reset];

% Plotting torques
torque = robot.rne(full_q, full_qd, full_qdd);
%plot_dynamics(torque, dt, full_q, full_qd, full_qdd); without drill load
plot_dynamics(full_trq, dt, full_q, full_qd, full_qdd);

%Displaying max speed and torque
disp("Max speed[rad/s] per joint:");
disp(max(abs(full_qd)));
disp("Max torque[Nm] per joint:");
disp(max(abs(full_trq)));


%% Trajectory helpers

% Between points

function [q, qd, qdd, trq] = between_traj2(P2, robot, s, t, dt)

    q0 = robot.getpos();
      

    %Base trajectory calculation
    phi1 = q0(1); 
    [phi2, ~] = cart2pol(P2(1, 4), P2(2, 4));

    phid = phi2 - phi1;
    
    q_base = q0 + s * [phid, 0, 0, 0, 0];
    [~, qd_base] = gradient(q_base, dt);
    [~, qdd_base] = gradient(qd_base, dt);

    %Rest of joints calculation
    q2 = ik5(robot, P2(1:3, 4), P2(1:3, 3), q_base(end, :));  
    [q_joints, qd_joints, qdd_joints] = jtraj(q_base(end, :), q2, t);
    
    %Summation and output
    q   = q_base + (q_joints - q_joints(1, :));
    qd  = qd_base + qd_joints;
    qdd = qdd_base + qdd_joints;
    trq = robot.rne(q, qd, qdd);
end



% Drilling motion

function [q, qd, qdd, trq] = drill_traj(P, A, D, robot, sd, sa, sr, seed, dt, drill_force, drill_moment)

    sd = min(max(sd, 0), 1); % Had errors with s-values being e^-13 bigger than 1.
    sa = min(max(sa, 0), 1);
    sr = min(max(sr, 0), 1);


    %Approach
    Ta = ctraj(P, A, sa); % Cartesian traj with approach interpolation
    N = size(Ta, 3); 
    for i = 1:N % Converting to angles
        q1(i, :) = ik5(robot, Ta(1:3, 4, i), Ta(1:3, 3, i), seed);
    end
    [~, qd1] = gradient(q1, dt); % Getting velocity and acceleration
    [~, qdd1] = gradient(qd1, dt);
    trq1 = robot.rne(q1, qd1, qdd1); % Torque calculation

    %Drilling
    Td = ctraj(A, D, sd); % Cartesian traj with drilling interpolation (constant speed)
    N = size(Td, 3);
    for i = 1:N % Converting to angles
        q2(i, :) = ik5(robot, Td(1:3, 4, i), Td(1:3, 3, i), q1(end, :));
    end
    [~, qd2] = gradient(q2, dt); %etc...
    [~, qdd2] = gradient(qd2, dt);
    trq2 = robot.rne(q2, qd2, qdd2, 'fext', [0, 0, drill_force, 0, 0, drill_moment]);

    %Retract
    Tr = ctraj(D, P, sr); % Cartesian traj with retraction interpolation
    N = size(Tr, 3);
    for i = 1:N
        q3(i, :) = ik5(robot, Tr(1:3, 4, i), Tr(1:3, 3, i), q2(end, :));
    end
    [~, qd3] = gradient(q3, dt);
    [~, qdd3] = gradient(qd3, dt);
    trq3 = robot.rne(q3, qd3, qdd3);

    %Output
    q = [q1; q2; q3];
    qd = [qd1; qd2; qd3];
    qdd = [qdd1; qdd2; qdd3];
    trq = [trq1; trq2; trq3];

end