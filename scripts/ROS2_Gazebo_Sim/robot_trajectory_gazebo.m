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
