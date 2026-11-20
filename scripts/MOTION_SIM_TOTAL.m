%% MOTION SIM TOTAL - Zero-position -> Drills 3 holes -> Zero-position
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

% Points on ceiling
p1_ceil = [0, -0.5];
p2_ceil = [0, 0.5];
p3_ceil = [0, 0.5];

points = [p1_ceil; p2_ceil];

% Tilt
tilt_mag = 15;
tilt_phi = 90;

P0 = double(robot.fkine(qz));

%% Time and interpolation
dt = 0.1; % Time step

%Between holes
Tb = 3; % Between holes motion time
Nb = round(Tb / dt) + 1; % Amount of samples (for between motion)
tb = (0:Nb-1)' * dt;
sb = tpoly(0, 1, tb);

%Drill approach
Ta = 3; %Approach time
Na = round(Ta / dt) + 1; %Amount of samples (for approach motion)
ta = (0:Na-1)' * dt; 
d = tpoly(0, CLEARANCE, ta, 0, DRILL_SPEED_UP);
sa = d / CLEARANCE;

%Drilling motion
% Vd = DRILL_SPEED_UP/DRILL_STROKE; % Fraction of motion / s
% Tacc = 0.5; %Acceleration and deceleration time
% Td = 1/Vd + Tacc; %Total motion time
% td = 0:dt:Td; % Time vector for trapezoidal interpolation
% Nd = length(td);
% sd = lspb(0, 1, td, Vd);
v = DRILL_SPEED_UP;
D = DRILL_STROKE;

T = ceil((D/v + 0.5/2) / dt) * dt;
t = (0:round(T/dt))' * dt;

t_brake = 2*(T - D/v);                % Braking duration
u = max(0, t - (T - t_brake));        % Time elapsed since braking began
d = v*t - v*u.^2/(2*t_brake);         % Distance along drill axis [m]
sd = d / D;

%Drill retract
Tr = 4; %Retract time
Nr = round(Tr / dt) + 1;
tr = (0:Nr-1)' * dt;
sr = tpoly(0, 1, tr);








% Plotting disc
plot_disc(points, CEILING, tilt_mag, tilt_phi);


%% Main

full_q = zeros(0, 5);
full_qd = zeros(0, 5);
full_qdd = zeros(0, 5);
full_trq = zeros(0, 5);
robot.plot(qz, 'view', [45 30]);

for i = 1:size(points, 1)
    P = ceiling_to_base(points(i, 1), points(i, 2), CEILING, CLEARANCE, tilt_mag, tilt_phi);  % Start pose
    A = ceiling_to_base(points(i, 1), points(i, 2), CEILING, 0, tilt_mag, tilt_phi);  % Approach pose (before drilling)
    D = ceiling_to_base(points(i, 1), points(i, 2), CEILING, -DRILL_STROKE, tilt_mag, tilt_phi);  % Final drill pose
    
    %Move to new point
    [q_between, qd_between, qdd_between, trq_between] = between_traj2(P, robot, sb, tb, dt);
    robot.plot(q_between);
    
    %Drill and retract
    [q_drill, qd_drill, qdd_drill, trq_drill] = drill_traj(P, A, D, robot, sd, sa, sr, qz, dt);
    robot.plot(q_drill);

    full_q = [full_q; q_between; q_drill];
    full_qd = [full_qd; qd_between; qd_drill];
    full_qdd = [full_qdd; qdd_between; qdd_drill];
    full_trq = [full_trq; trq_between; trq_drill];

end

[q_reset, qd_reset, qdd_reset] = between_traj2(double(robot.fkine(qz)), robot, sb, tb, dt);
trq_reset = robot.rne(q_reset, qd_reset, qdd_reset);
robot.plot(q_reset);


full_q = [full_q; q_reset];
full_qd = [full_qd; qd_reset];
full_qdd = [full_qdd; qdd_reset];
full_trq = [full_trq; trq_reset];

%Plotting torques
torque = robot.rne(full_q, full_qd, full_qdd);
plot_dynamics(torque, dt, full_q, full_qd, full_qdd);
plot_dynamics(full_trq, dt, full_q, full_qd, full_qdd);



%% Trajectory helpers

% Between points

function [q, qd, qdd, trq] = between_traj2(P2, robot, s, t, dt)

    q0 = robot.getpos();
      

    %Base trajectory calculation
    phi1 = q0(1); 
    [phi2, ~] = cart2pol(P2(1, 4), P2(2, 4));

    %Find angle for base + directly move robot to that REWRITE!!!
    % if abs(phi1 - phi2) > pi/2
    %     phi2 = wrap(phi2 + pi);
    % end

    phid = phi2 - phi1;
    
    q_base = q0 + s * [phid, 0, 0, 0, 0];
    [~, qd_base] = gradient(q_base, dt);
    [~, qdd_base] = gradient(qd_base, dt);

    %Rest of joints calculation
    q2 = ik5(robot, P2(1:3, 4), P2(1:3, 3), q_base(end, :));  
    [q_joints, qd_joints, qdd_joints] = jtraj(q_base(end, :), q2, t);
    
    %Outputs
    q = [q_base(:, 1), q_joints(:, 2:5)];
    qd  = [qd_base(:,1),  qd_joints(:,2:5)];
    qdd = [qdd_base(:,1), qdd_joints(:,2:5)];
    trq = robot.rne(q, qd, qdd);
    %q = [q_base; q_joints];
    %qd = [qd_base; qd_joints];
    %qdd = [qdd_base; qdd_joints];

    %traj = [base_traj(:, 1), traj_joints(:, 2:5)];

end

% Drilling motion

function [q, qd, qdd, trq] = drill_traj(P, A, D, robot, sd, sa, sr, seed, dt)
    sd = min(max(sd, 0), 1);
    sa = min(max(sa, 0), 1);
    sr = min(max(sr, 0), 1);


    %Approach
    Ta = ctraj(P, A, sa);
    N = size(Ta, 3);
    for i = 1:N
        q1(i, :) = ik5(robot, Ta(1:3, 4, i), Ta(1:3, 3, i), seed);
    end
    [~, qd1] = gradient(q1, dt);
    [~, qdd1] = gradient(qd1, dt);
    trq1 = robot.rne(q1, qd1, qdd1);

    %Drilling
    Td = ctraj(A, D, sd);
    N = size(Td, 3);
    for i = 1:N
        q2(i, :) = ik5(robot, Td(1:3, 4, i), Td(1:3, 3, i), seed);
    end
    [~, qd2] = gradient(q2, dt);
    [~, qdd2] = gradient(qd2, dt);
    trq2 = robot.rne(q2, qd2, qdd2, 'fext', [0, 0, 50, 0, 0, 0]);

    %Retract
    Tr = ctraj(D, P, sr);
    N = size(Tr, 3);
    for i = 1:N
        q3(i, :) = ik5(robot, Tr(1:3, 4, i), Tr(1:3, 3, i), seed);
    end
    [~, qd3] = gradient(q3, dt);
    [~, qdd3] = gradient(qd3, dt);
    trq3 = robot.rne(q3, qd3, qdd3);

    %Output
    q = [q1; q2; q3];
    qd = [qd1; qd2; qd3];
    qdd = [qdd1; qdd2; qdd3];
    trq = [trq1; trq2; trq3];


    % Tfull = cat(3, Ta, Td(:,:,2:end), Tr(:,:,2:end)); 
    % for i = 1:N
    %     q(i, :) = ik5(robot, Tfull(1:3, 4, i), Tfull(1:3, 3, i), seed);
    % end
    % 
    % [~, qd] = gradient(q, dt);
    % [~, qdd] = gradient(qd, dt);

end