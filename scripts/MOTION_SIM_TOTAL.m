%% MOTION SIM TOTAL - Zero-position -> Drills 3 holes -> Zero-position
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

% Points on ceiling
p1_ceil = [0.1, 0.2];
p2_ceil = [0.2, -0.3];
p3_ceil = [0, 0.5];

points = p1_ceil;

% Tilt
tilt_mag = 15;
tilt_phi = 0;

P0 = double(robot.fkine(qz));

% Time
dt = 0.1; % Time step

Tb = 3; % Between holes motion time
Nb = Tb / dt; % Amount of samples (for between motion)

Vd = DRILL_SPEED_UP/DRILL_STROKE; 
Tacc = 0.5;
Td = 1/Vd + Tacc;
td = 0:dt:Td; % Time vector for trapezoidal interpolation
Nd = length(td);


sb = tpoly(0, 1, Nb);
sd = lspb(0, 1, td, Vd);


% Plotting disc
plot_disc(points, CEILING, tilt_mag, tilt_phi);


%% Main

robot.plot(qz);

for i = 1:size(points, 1)
    P = ceiling_to_base(points(i, 1), points(i, 2), CEILING, CLEARANCE, tilt_mag, tilt_phi);  % Start pose
    A = ceiling_to_base(points(i, 1), points(i, 2), CEILING, -APPROACH_DIST, tilt_mag, tilt_phi);  % Approach pose (before drilling)
    D = ceiling_to_base(points(i, 1), points(i, 2), CEILING, -DRILL_STROKE, tilt_mag, tilt_phi);  % Final drill pose

    traj = between_traj2(P, robot, sb, Nb)
    robot.plot(traj);

    traj = drill_traj(P, D, robot, sd, Nd, qz);
    robot.plot(traj);
    traj = drill_traj(D, P, robot, sd, Nd, qz);
    robot.plot(traj);

end

traj = between_traj2(double(robot.fkine(qz)), robot, sb, Nb);
robot.plot(traj);





%% Trajectory helpers

% Between points

function traj = between_traj2(P2, robot, s, N)

    q1 = robot.getpos();
      

    %Get azimuth of both points
    phi1 = q1(1); 
    [phi2, ~] = cart2pol(P2(1, 4), P2(2, 4));

    %Find angle for base + directly move robot to that REWRITE!!!
    % if abs(phi1 - phi2) > pi/2
    %     phi2 = wrap(phi2 + pi);
    % end

    phid = phi2 - phi1;
    
    base_traj = q1 + s * [phid, 0, 0, 0, 0];
    q_last = base_traj(N);
    q2 = ik5(robot, P2(1:3, 4), P2(1:3, 3), q_last);  
    traj_joints = jtraj(base_traj(N, :), q2, N);
    
    traj = [base_traj; traj_joints];

    %traj = [base_traj(:, 1), traj_joints(:, 2:5)];

end

% Drilling motion

function traj = drill_traj(P, D, robot, s, N, seed)
  
Ttraj = ctraj(P, D, s);
    for i = 1:N
        traj(i, :) = ik5(robot, Ttraj(1:3, 4, i), Ttraj(1:3, 3, i), seed);
    end
    
end