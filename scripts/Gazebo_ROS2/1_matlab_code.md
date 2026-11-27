# MATLAB code

## MATLAB/circle_gazebo.m

```matlab
%% CIRCLE + GAZEBO
% The tool follows the outer edge of the ceiling disc (radius 0.5 m),
% once all the way around. Every step is drawn in MATLAB AND sent to
% Gazebo at the same time.
%
% HOW TO RUN:
%   1. Save this file in the folder that has ik5.m, ceiling_to_base.m,
%      config.m and robot_model.m:
%        C:\Users\adria\Downloads\zip_til_adrian (1)\zip_til_adrian
%   2. In MATLAB, double-click that folder in "Current Folder".
%   3. Start Gazebo + controllers, then press Run.

clear; clc; close all;

config;        % your config.m (CEILING, CLEARANCE)
robot_model;   % creates "robot"
q_zero = zeros(1, 5);

% Used only if your config.m doesn't define them
if ~exist('CEILING',   'var'), CEILING   = 1.5;  end  % ceiling height above robot base (m)
if ~exist('CLEARANCE', 'var'), CLEARANCE = 0.05; end  % tool distance below ceiling (m)

%% Circle settings (edit these)
disc_r   = 0.5;    % circle radius on the ceiling (m)
tilt_mag = 10;     % ceiling tilt (deg)
tilt_phi = 0;      % tilt direction (deg)

T_circle = 15;     % time for one full circle (s)
N_circle = 150;    % number of points on the circle
T_move   = 4;      % time to move from zero to the circle, and back (s)
dt       = T_circle / N_circle;

%% Connect to Gazebo through ROS 2
node = ros2node('/matlab_robot', 24);
pub  = ros2publisher(node, '/joint_position_controller/commands', ...
                     'std_msgs/Float64MultiArray');
msg  = ros2message(pub);
pause(2);   % ROS 2 discovery

%% Circle points on the ceiling
% Goes from -180 to +180 deg, so joint 1 sweeps its whole range once
% without jumping (Gazebo joint 1 limit is +-180 deg).
a = linspace(-pi + 0.02, pi - 0.02, N_circle);

Tc = zeros(4, 4, N_circle);
for k = 1:N_circle
    Tc(:,:,k) = ceiling_to_base(disc_r*cos(a(k)), disc_r*sin(a(k)), ...
                                CEILING, CLEARANCE, tilt_mag, tilt_phi);
end

%% Inverse kinematics for every point (seeded from the previous point)
fprintf('Calculating inverse kinematics...\n');
Q = zeros(N_circle, 5);
seed = [a(1), 0, 0, 0, 0];
for k = 1:N_circle
    Q(k,:) = ik5(robot, Tc(1:3,4,k), Tc(1:3,3,k), seed);
    seed = Q(k,:);
end
Q = unwrap(Q);   % remove any +-2*pi jumps between points

if any(abs(Q(:)) > pi)
    warning('Some joint angles are outside +-180 deg; Gazebo will clip them.');
end

%% Plot the disc and the robot
figure; hold on; grid on; axis equal;
edge = squeeze(Tc(1:3, 4, :));
fill3(edge(1,:), edge(2,:), edge(3,:), [0.8 0.8 0.8], 'FaceAlpha', 0.4);
plot3(edge(1,:), edge(2,:), edge(3,:), 'r-', 'LineWidth', 1.5);
robot.plot(q_zero);

%% Run: zero -> circle start -> around the circle -> zero
Nm = round(T_move / 0.1);

fprintf('Moving to zero...\n');
send_and_plot(robot, pub, msg, q_zero, 1.0);

fprintf('Moving to circle start...\n');
send_and_plot(robot, pub, msg, jtraj(q_zero, Q(1,:), Nm), 0.1);

fprintf('Following the circle...\n');
send_and_plot(robot, pub, msg, Q, dt);

fprintf('Returning to zero...\n');
send_and_plot(robot, pub, msg, jtraj(Q(end,:), q_zero, Nm), 0.1);

fprintf('Done.\n');


%% ===================== Helper =====================
function send_and_plot(robot, pub, msg, Q, dt)
% Send each row of Q to Gazebo and draw the same pose in MATLAB.
    for k = 1:size(Q, 1)
        q = Q(k, :);
        msg.data = q;
        send(pub, msg);
        robot.animate(q);
        drawnow;
        pause(dt);
    end
end

```

## MATLAB/fit_ceiling_to_tool.m

```matlab
%% FIT CEILING TO TOOL
% Run this RIGHT AFTER circle_gazebo.m (it uses Q from the workspace).
%
% It computes the tool-tip positions exactly the way Gazebo does (the URDF
% joint chain with the 0.35 m tool), fits a plane through them, and prints
% one command for the Ubuntu VM that moves the ceiling plate so its
% underside touches the tool tip.

if ~exist('Q', 'var')
    error('Run circle_gazebo first, so Q (the circle joint angles) exists.');
end

% Gazebo/URDF forward kinematics (same numbers as my_robot.urdf.xacro)
Rz = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];
Rx = @(t) [1 0 0 0; 0 cos(t) -sin(t) 0; 0 sin(t) cos(t) 0; 0 0 0 1];
Tr = @(x,y,z) [eye(3) [x;y;z]; 0 0 0 1];
h  = pi/2;
fk = @(q) Rz(q(1))*Tr(0,0,0.15)*Rx(-h)*Rz(q(2))*Tr(0.6,0,0)*Rz(q(3))*Rx(-h) ...
        *Rz(q(4))*Tr(0,0,0.6)*Rx(h)*Rz(q(5))*Rx(-h)*Tr(0,0,0.35);

% Tool-tip positions on the circle (relative to base_link)
N = size(Q, 1);
tips = zeros(3, N);
for k = 1:N
    T = fk(Q(k,:));
    tips(:, k) = T(1:3, 4);
end

% Best-fit plane through the tips
c = mean(tips, 2);
[~, ~, V] = svd((tips - c)', 0);
n = V(:, 3);
if n(3) < 0, n = -n; end            % normal points up, towards the ceiling

fprintf('Tip distance from fitted plane: max %.1f mm\n', 1000*max(abs(n'*(tips - c))));

% Plate frame: z = plane normal, x = base x projected onto the plane
x = [1;0;0] - n*(n'*[1;0;0]);  x = x / norm(x);
y = cross(n, x);
R = [x y n];

thickness = 0.01;
p = c + n * thickness/2;            % underside of the plate touches the tips

pitch = -asin(max(-1, min(1, R(3,1))));
roll  = atan2(R(3,2), R(3,3));
yaw   = atan2(R(2,1), R(1,1));

origin = sprintf('<origin xyz="%.4f %.4f %.4f" rpy="%.5f %.5f %.5f"/>', p, roll, pitch, yaw);

fprintf('\nNew ceiling origin:\n  %s\n\n', origin);
fprintf('Copy this ONE line into the Ubuntu terminal:\n\n');
fprintf(['sed -i ''/name="base_to_ceiling"/,/<\\/joint>/ s|<origin .*/>|%s|'' ' ...
         '~/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro\n\n'], origin);

```

## MATLAB/make_ceiling_urdf.m

```matlab
%% MAKE CEILING URDF
% Prints a ready-to-paste URDF block for a flat ceiling plate in Gazebo,
% placed exactly where your MATLAB ceiling is (same CEILING height and
% tilt as circle_gazebo.m, using your own ceiling_to_base.m).
%
% Run it from the zip_til_adrian folder, then copy the printed text into
% my_robot.urdf.xacro (just before the final </robot> line).

clear; clc;
config;

if ~exist('CEILING', 'var'), CEILING = 1.5; end   % same default as circle_gazebo.m

% --- keep these the same as in circle_gazebo.m ---
tilt_mag = 10;     % deg
tilt_phi = 0;      % deg
plate_size = 1.4;  % m, side length of the square plate
% --------------------------------------------------

% Pose of the ceiling centre, relative to the robot base
T = ceiling_to_base(0, 0, CEILING, 0, tilt_mag, tilt_phi);
T = double(T);
R = T(1:3, 1:3);
p = T(1:3, 4);

% URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
pitch = -asin(max(-1, min(1, R(3,1))));
roll  = atan2(R(3,2), R(3,3));
yaw   = atan2(R(2,1), R(1,1));

fprintf('\n<!-- ===== Ceiling plate (visual only, generated by make_ceiling_urdf.m) ===== -->\n');
fprintf('<link name="ceiling_link">\n');
fprintf('  <inertial>\n');
fprintf('    <mass value="0.1"/>\n');
fprintf('    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>\n');
fprintf('  </inertial>\n');
fprintf('  <visual>\n');
fprintf('    <geometry>\n');
fprintf('      <box size="%.3f %.3f 0.01"/>\n', plate_size, plate_size);
fprintf('    </geometry>\n');
fprintf('    <material name="ceiling_white">\n');
fprintf('      <color rgba="0.9 0.9 0.9 0.6"/>\n');
fprintf('    </material>\n');
fprintf('  </visual>\n');
fprintf('</link>\n');
fprintf('<joint name="base_to_ceiling" type="fixed">\n');
fprintf('  <parent link="base_link"/>\n');
fprintf('  <child link="ceiling_link"/>\n');
fprintf('  <origin xyz="%.4f %.4f %.4f" rpy="%.5f %.5f %.5f"/>\n', p(1), p(2), p(3), roll, pitch, yaw);
fprintf('</joint>\n');
fprintf('<gazebo reference="ceiling_link">\n');
fprintf('  <gravity>false</gravity>\n');
fprintf('</gazebo>\n\n');

```

## MATLAB/matlab_ros2_connection_test.m

```matlab
%% Simple MATLAB -> Gazebo ROS 2 connection test
% Run only when Gazebo and joint_position_controller are active.
% The command moves joint 1 by 0.2 rad (about 11.5 degrees).

node = ros2node('/matlab_robot_test', 24);

pub = ros2publisher( ...
    node, ...
    '/joint_position_controller/commands', ...
    'std_msgs/Float64MultiArray');

msg = ros2message(pub);
pause(2);

msg.data = [0.2, 0, 0, 0, 0];
send(pub, msg);

fprintf('Sent test joint command to Gazebo.\n');

```

## MATLAB/motion_sim_gazebo.m

```matlab
%% MOTION SIM + GAZEBO
% Zero position -> drill holes in the ceiling -> back to zero position.
% Same motion as your original MOTION SIM TOTAL script, but every step is
% drawn in MATLAB AND sent to Gazebo at the same time.
%
% HOW TO RUN:
%   1. Save this file in the folder that has ik5.m, ceiling_to_base.m,
%      config.m and robot_model.m:
%        C:\Users\adria\Downloads\zip_til_adrian (1)\zip_til_adrian
%   2. In MATLAB, cd into that folder (or double-click it in "Current Folder").
%   3. Start Gazebo + controllers, then press Run.

clear; clc; close all;

config;        % your config.m (CEILING, CLEARANCE, ...)
robot_model;   % your robot model (creates "robot")
q_zero = zeros(1, 5);   % Zero joint pose

%% Drilling settings
% Your config.m doesn't define these, so defaults are set here.
% EDIT THESE to match your PDR. Values in config.m are used if they exist.
if ~exist('DRILL_STROKE',   'var'), DRILL_STROKE   = 0.03; end  % hole depth (m)
if ~exist('DRILL_SPEED_UP', 'var'), DRILL_SPEED_UP = 0.01; end  % drill feed speed (m/s)
if ~exist('APPROACH_DIST',  'var'), APPROACH_DIST  = 0.02; end  % (m), not used in motion
if ~exist('CEILING',        'var'), CEILING        = 1.5;  end  % ceiling height above robot base (m)
if ~exist('CLEARANCE',      'var'), CLEARANCE      = 0.05; end  % start distance below ceiling (m)

%% Holes to drill (ceiling coordinates)
p1_ceil = [0.1, 0.2];
p2_ceil = [0.2, -0.3];
p3_ceil = [0, 0.5];

points = [p1_ceil; p2_ceil; p3_ceil];   % all three holes

% Tilt
tilt_mag = 15;
tilt_phi = 0;

%% Timing
dt = 0.1;                 % Time step (s), also the Gazebo send interval
Tb = 3;                   % Time for each move between holes (s)
Nb = round(Tb / dt);      % Samples per between-move (round avoids 29.999...)

Vd   = DRILL_SPEED_UP / DRILL_STROKE;
Tacc = 0.5;
Td   = 1/Vd + Tacc;
Nd   = round(Td / dt) + 1;
td   = linspace(0, Td, Nd);

sb = tpoly(0, 1, Nb);
sd = lspb(0, 1, td, Vd);

%% Connect to Gazebo through ROS 2
node = ros2node('/matlab_robot', 24);
pub  = ros2publisher(node, '/joint_position_controller/commands', ...
                     'std_msgs/Float64MultiArray');
msg  = ros2message(pub);
pause(2);   % ROS 2 discovery

%% Plot disc and robot at zero
figure; hold on; grid on; axis equal;
draw_disc(points, CEILING, tilt_mag, tilt_phi);
robot.plot(q_zero);

% Make sure Gazebo starts at zero too
send_and_plot(robot, pub, msg, q_zero, 1.0);
q_now = q_zero;

%% Main: drill each hole
for i = 1:size(points, 1)
    fprintf('Hole %d of %d\n', i, size(points, 1));

    P = ceiling_to_base(points(i,1), points(i,2), CEILING,  CLEARANCE,    tilt_mag, tilt_phi); % start pose
    D = ceiling_to_base(points(i,1), points(i,2), CEILING, -DRILL_STROKE, tilt_mag, tilt_phi); % drill end pose

    % Rotate base, then move the arm to the start pose below the hole
    traj = between_traj(q_now, P, robot, sb, Nb);
    send_and_plot(robot, pub, msg, traj, dt);
    q_now = traj(end, :);

    % Drill up into the ceiling
    traj = drill_traj(P, D, robot, sd, Nd, q_now);
    send_and_plot(robot, pub, msg, traj, dt);
    q_now = traj(end, :);

    % Pull back down
    traj = drill_traj(D, P, robot, sd, Nd, q_now);
    send_and_plot(robot, pub, msg, traj, dt);
    q_now = traj(end, :);
end

%% Back to zero position
fprintf('Returning to zero position\n');
traj = jtraj(q_now, q_zero, Nb);
send_and_plot(robot, pub, msg, traj, dt);

fprintf('Motion finished.\n');


%% ===================== Helpers =====================

function draw_disc(points, CEILING, tilt_mag, tilt_phi)
% Grey 0.5 m disc on the ceiling with the holes marked in red.
    a = linspace(0, 2*pi, 60);
    edge = zeros(3, numel(a));
    for k = 1:numel(a)
        Tp = ceiling_to_base(0.5*cos(a(k)), 0.5*sin(a(k)), CEILING, 0, tilt_mag, tilt_phi);
        edge(:, k) = Tp(1:3, 4);
    end
    fill3(edge(1,:), edge(2,:), edge(3,:), [0.8 0.8 0.8], 'FaceAlpha', 0.4);
    for k = 1:size(points, 1)
        Tp = ceiling_to_base(points(k,1), points(k,2), CEILING, 0, tilt_mag, tilt_phi);
        plot3(Tp(1,4), Tp(2,4), Tp(3,4), 'ro', 'MarkerFaceColor', 'r');
    end
end

function send_and_plot(robot, pub, msg, Q, dt)
% Send each row of Q to Gazebo and draw the same pose in MATLAB.
    for k = 1:size(Q, 1)
        q = Q(k, :);
        msg.data = q;
        send(pub, msg);
        robot.animate(q);
        drawnow;
        pause(dt);
    end
end

function traj = between_traj(q1, P2, robot, s, N)
% Rotate the base towards the next hole, then move the other joints to it.
% q1 is passed in instead of robot.getpos(), so it always matches Gazebo.
    [phi2, ~] = cart2pol(P2(1,4), P2(2,4));
    phid = phi2 - q1(1);

    base_traj = q1 + s * [phid, 0, 0, 0, 0];
    q_last = base_traj(N, :);                       % full joint row
    q2 = ik5(robot, P2(1:3,4), P2(1:3,3), q_last);
    traj_joints = jtraj(q_last, q2, N);

    traj = [base_traj; traj_joints];
end

function traj = drill_traj(P, D, robot, s, N, seed)
% Straight-line drill motion. Each IK step is seeded with the previous
% solution so the arm can't suddenly flip to another configuration.
    Ttraj = ctraj(P, D, s);
    traj = zeros(N, 5);
    for i = 1:N
        traj(i, :) = ik5(robot, Ttraj(1:3,4,i), Ttraj(1:3,3,i), seed);
        seed = traj(i, :);
    end
end

```

## MATLAB/robot_model.m

```matlab
%% Robot DH model
% Reconstructed from the DH parameters provided in the conversation.
% Requires Peter Corke's Robotics Toolbox for MATLAB.

lb = 0.15;   % Base height (m)
L  = 0.60;   % Main link length (m)
tl = 0.35;   % Tool offset (m)

T_tool = eye(4);
T_tool(3,4) = tl;

% Standard DH parameters
Links(1) = Link('d', lb, 'a', 0,  'alpha', -pi/2);
Links(2) = Link('d', 0,  'a', L,  'alpha', 0);
Links(3) = Link('d', 0,  'a', 0,  'alpha', -pi/2);
Links(4) = Link('d', L,  'a', 0,  'alpha', pi/2);
Links(5) = Link('d', 0,  'a', 0,  'alpha', -pi/2);

robot = SerialLink(Links, 'name', 'myRobot', 'tool', T_tool);

% Zero joint configuration used as an initial IK guess.
qz = zeros(1,5);
q0 = qz;

```

## MATLAB/robot_trajectory_gazebo.m

```matlab
%% Robot trajectory and Gazebo control
% Two modes:
%   mode = "demo"       : move through a set of joint poses
%   mode = "trajectory" : run the ceiling-to-ceiling trajectory calculation
%
% Every motion is sent to Gazebo AND drawn in MATLAB at the same time,
% one sample at a time, so both robots move together.
%
% Requirements:
%   - robot_model.m in the MATLAB path
%   - Peter Corke's Robotics Toolbox for MATLAB
%   - MATLAB ROS 2 support
%   - Gazebo simulation with joint_position_controller active
%   - Trajectory mode also needs config.m, ceiling_to_base.m and ik5.m

clear;
clc;
close all;

%% Select mode
mode = "demo";       % Change to "trajectory" for the ceiling trajectory

%% Create the robot model
robot_model;
q_zero = zeros(1, 5);   % Zero joint pose (defined here, not in robot_model)

%% Connect MATLAB to the ROS 2 network used by Gazebo
domainID = 24;
node = ros2node('/matlab_robot', domainID);

pub = ros2publisher( ...
    node, ...
    '/joint_position_controller/commands', ...
    'std_msgs/Float64MultiArray');

msg = ros2message(pub);
pause(2);  % Allow ROS 2 discovery/publisher setup

%% Draw the robot once at the zero pose (later calls only animate it)
figure;
robot.plot(q_zero);
hold on;

%% DEMO MODE
if mode == "demo"
    fprintf('Running demo...\n');

    % Each row is [joint1 joint2 joint3 joint4 joint5], in degrees.
    demo_poses_deg = [ ...
          0    0    0    0    0;
         45   20  -30   20   30;
        -45  -30   40  -30  -40;
         60   40  -50   30   60;
        -60  -40   50  -30  -60;
          0    0    0    0    0];
    demo_poses = deg2rad(demo_poses_deg);

    % Time per move. Joint velocity limit in the URDF is 1 rad/s, so keep
    % this long enough that Gazebo can actually keep up.
    T_move = 4;     % seconds
    N_move = 40;    % samples per move

    for i = 2:size(demo_poses, 1)
        fprintf('Moving to demo pose %d of %d\n', i, size(demo_poses, 1));
        Q = jtraj(demo_poses(i-1, :), demo_poses(i, :), N_move);
        send_and_plot(robot, pub, msg, Q, T_move / N_move);
    end

    fprintf('Demo finished.\n');
    return;
end

%% TRAJECTORY MODE
config;

% Target points in the ceiling coordinate system
p1_ceil = [0.5, 0];
p2_ceil = [-0.5, 0];

tilt_mag = 10;
tilt_phi = 0;

T1 = ceiling_to_base( ...
    p1_ceil(1), p1_ceil(2), CEILING, CLEARANCE, tilt_mag, tilt_phi);
T2 = ceiling_to_base( ...
    p2_ceil(1), p2_ceil(2), CEILING, -CLEARANCE, tilt_mag, tilt_phi);

% Trajectory settings
T = 3;       % seconds
N = 50;      % number of samples
dt = T / N;
s = tpoly(0, 1, N);

%% Plot the circular working area
disc_r = 0.5;
a = linspace(0, 2*pi, 60);
edge = zeros(3, numel(a));

for i = 1:numel(a)
    Tp = ceiling_to_base( ...
        disc_r*cos(a(i)), disc_r*sin(a(i)), ...
        CEILING, 0, tilt_mag, tilt_phi);
    edge(:, i) = Tp(1:3, 4);
end

fill3(edge(1,:), edge(2,:), edge(3,:), [0.8 0.8 0.8], 'FaceAlpha', 0.4);
plot3(T1(1,4), T1(2,4), T1(3,4), 'go', 'MarkerFaceColor', 'g');
plot3(T2(1,4), T2(2,4), T2(3,4), 'ro', 'MarkerFaceColor', 'r');

%% Stage 0: move from the zero pose to the starting configuration
current = ik5(robot, T1(1:3, 4), T1(1:3, 3), q_zero);

fprintf('Moving to start pose...\n');
send_and_plot(robot, pub, msg, jtraj(q_zero, current, 40), 0.1);

%% Stage 1: rotate the robot base
[phi1, ~] = cart2pol(T1(1,4), T1(2,4));
[phi2, ~] = cart2pol(T2(1,4), T2(2,4));
phid = phi2 - phi1;

base_traj = current + s * [phid, 0, 0, 0, 0];

fprintf('Rotating base...\n');
send_and_plot(robot, pub, msg, base_traj, dt);

%% Stage 2: move the remaining joints to the end configuration
q1 = base_traj(end, :);
q2 = ik5(robot, T2(1:3, 4), T2(1:3, 3), q1);

traj_joints = jtraj(q1, q2, N);

fprintf('Moving to end pose...\n');
send_and_plot(robot, pub, msg, traj_joints, dt);

fprintf('Ceiling trajectory finished.\n');

%% ===================== Local function =====================
function send_and_plot(robot, pub, msg, Q, dt)
% Send each row of Q to Gazebo and draw the same pose in MATLAB.
    for k = 1:size(Q, 1)
        q = Q(k, :);
        msg.data = q;
        send(pub, msg);
        robot.animate(q);
        drawnow;
        pause(dt);
    end
end

```
