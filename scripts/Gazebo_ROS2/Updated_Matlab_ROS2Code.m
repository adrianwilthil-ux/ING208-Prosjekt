%% ROBOT ALL-IN-ONE (MATLAB + Gazebo over ROS 2)
% All MATLAB code for the 5-joint overhead drilling arm, combined into one file:
%   robot_trajectory_gazebo.m, circle_gazebo.m, motion_sim_gazebo.m,
%   fit_ceiling_to_tool.m, make_ceiling_urdf.m, matlab_ros2_connection_test.m
%   and robot_model.m (built in as the local function make_robot).
%
% HOW TO RUN:
%   1. Put this file in the folder that has ik5.m, ceiling_to_base.m and config.m:
%        C:\Users\adria\Downloads\zip_til_adrian (1)\zip_til_adrian
%      (Those three files are NOT included here. "demo" and "test" modes
%       work without them.)
%   2. Pick a mode below.
%   3. Start Gazebo + controllers in the Ubuntu VM, then press Run.
%
% Requirements: Peter Corke's Robotics Toolbox, MATLAB ROS 2 support,
% Gazebo with joint_position_controller active (ROS_DOMAIN_ID = 24).

clear; clc; close all;

%% ===================== SELECT MODE =====================
mode = "demo";
%   "demo"         : move through a set of joint poses       (no helpers needed)
%   "trajectory"   : ceiling point to ceiling point           (needs ik5, ceiling_to_base, config)
%   "circle"       : tool traces the 0.5 m ceiling circle     (needs ik5, ceiling_to_base, config)
%   "drill"        : drill 3 holes in the ceiling             (needs ik5, ceiling_to_base, config)
%   "ceiling_urdf" : print URDF block for the ceiling plate   (needs ceiling_to_base, config; no ROS)
%   "test"         : send one test command (joint 1 = 0.2 rad) (no helpers needed)

FIT_CEILING_AFTER_CIRCLE = true;   % "circle" mode: also print the ceiling-fit sed command

DOMAIN_ID = 24;
TOPIC     = '/joint_position_controller/commands';

%% ===================== RUN =====================
needs_ros = mode ~= "ceiling_urdf";
if needs_ros
    node = ros2node('/matlab_robot', DOMAIN_ID);
    pub  = ros2publisher(node, TOPIC, 'std_msgs/Float64MultiArray');
    msg  = ros2message(pub);
    pause(2);   % ROS 2 discovery
end

switch mode
    case "demo"
        run_demo(make_robot(), pub, msg);
    case "trajectory"
        run_trajectory(make_robot(), pub, msg);
    case "circle"
        Q = run_circle(make_robot(), pub, msg);   % Q stays in the workspace
        if FIT_CEILING_AFTER_CIRCLE
            fit_ceiling_to_tool(Q);
        end
    case "drill"
        run_drill(make_robot(), pub, msg);
    case "ceiling_urdf"
        make_ceiling_urdf();
    case "test"
        msg.data = [0.2, 0, 0, 0, 0];
        send(pub, msg);
        fprintf('Sent test joint command to Gazebo.\n');
    otherwise
        error('Unknown mode "%s".', mode);
end


%% =====================================================================
%% ========================= LOCAL FUNCTIONS ===========================
%% =====================================================================

%% ---------- Robot model (was robot_model.m) ----------
function robot = make_robot()
% Standard DH model. Matches the URDF in Gazebo exactly.
    lb = 0.15;   % Base height (m)
    L  = 0.60;   % Main link length (m)
    tl = 0.35;   % Tool offset (m)

    T_tool = eye(4);
    T_tool(3,4) = tl;

    Links(1) = Link('d', lb, 'a', 0,  'alpha', -pi/2);
    Links(2) = Link('d', 0,  'a', L,  'alpha', 0);
    Links(3) = Link('d', 0,  'a', 0,  'alpha', -pi/2);
    Links(4) = Link('d', L,  'a', 0,  'alpha', pi/2);
    Links(5) = Link('d', 0,  'a', 0,  'alpha', -pi/2);

    robot = SerialLink(Links, 'name', 'myRobot', 'tool', T_tool);
end

%% ---------- Settings from config.m (with defaults) ----------
function [CEILING, CLEARANCE, DRILL_STROKE, DRILL_SPEED_UP] = load_config()
% Runs your config.m and fills in defaults for anything it doesn't define.
    config;
    if ~exist('CEILING',        'var'), CEILING        = 1.5;  end  % ceiling height above base (m)
    if ~exist('CLEARANCE',      'var'), CLEARANCE      = 0.05; end  % tool distance below ceiling (m)
    if ~exist('DRILL_STROKE',   'var'), DRILL_STROKE   = 0.03; end  % hole depth (m)
    if ~exist('DRILL_SPEED_UP', 'var'), DRILL_SPEED_UP = 0.01; end  % drill feed speed (m/s)
end

%% ---------- Mode: demo (was robot_trajectory_gazebo.m, demo part) ----------
function run_demo(robot, pub, msg)
    figure; robot.plot(zeros(1,5)); hold on;
    fprintf('Running demo...\n');

    % Each row is [joint1 joint2 joint3 joint4 joint5], in degrees.
    demo_poses = deg2rad([ ...
          0    0    0    0    0;
         45   20  -30   20   30;
        -45  -30   40  -30  -40;
         60   40  -50   30   60;
        -60  -40   50  -30  -60;
          0    0    0    0    0]);

    T_move = 4;     % seconds per move (URDF velocity limit is 1 rad/s)
    N_move = 40;    % samples per move

    for i = 2:size(demo_poses, 1)
        fprintf('Moving to demo pose %d of %d\n', i, size(demo_poses, 1));
        Q = jtraj(demo_poses(i-1,:), demo_poses(i,:), N_move);
        send_and_plot(robot, pub, msg, Q, T_move / N_move);
    end
    fprintf('Demo finished.\n');
end

%% ---------- Mode: trajectory (was robot_trajectory_gazebo.m, trajectory part) ----------
function run_trajectory(robot, pub, msg)
    [CEILING, CLEARANCE] = load_config();
    q_zero = zeros(1,5);

    p1_ceil = [0.5, 0];
    p2_ceil = [-0.5, 0];
    tilt_mag = 10;
    tilt_phi = 0;

    T1 = ceiling_to_base(p1_ceil(1), p1_ceil(2), CEILING,  CLEARANCE, tilt_mag, tilt_phi);
    T2 = ceiling_to_base(p2_ceil(1), p2_ceil(2), CEILING, -CLEARANCE, tilt_mag, tilt_phi);

    T = 3; N = 50; dt = T / N;
    s = tpoly(0, 1, N);

    figure; robot.plot(q_zero); hold on;
    draw_disc(zeros(0,2), CEILING, tilt_mag, tilt_phi);
    plot3(T1(1,4), T1(2,4), T1(3,4), 'go', 'MarkerFaceColor', 'g');
    plot3(T2(1,4), T2(2,4), T2(3,4), 'ro', 'MarkerFaceColor', 'r');

    % Stage 0: zero -> start configuration
    current = ik5(robot, T1(1:3,4), T1(1:3,3), q_zero);
    fprintf('Moving to start pose...\n');
    send_and_plot(robot, pub, msg, jtraj(q_zero, current, 40), 0.1);

    % Stage 1: rotate base
    [phi1, ~] = cart2pol(T1(1,4), T1(2,4));
    [phi2, ~] = cart2pol(T2(1,4), T2(2,4));
    base_traj = current + s * [phi2 - phi1, 0, 0, 0, 0];
    fprintf('Rotating base...\n');
    send_and_plot(robot, pub, msg, base_traj, dt);

    % Stage 2: remaining joints to end configuration
    q1 = base_traj(end,:);
    q2 = ik5(robot, T2(1:3,4), T2(1:3,3), q1);
    fprintf('Moving to end pose...\n');
    send_and_plot(robot, pub, msg, jtraj(q1, q2, N), dt);

    fprintf('Ceiling trajectory finished.\n');
end

%% ---------- Mode: circle (was circle_gazebo.m) ----------
function Q = run_circle(robot, pub, msg)
    [CEILING, CLEARANCE] = load_config();
    q_zero = zeros(1,5);

    disc_r   = 0.5;    % circle radius on the ceiling (m)
    tilt_mag = 10;     % ceiling tilt (deg)
    tilt_phi = 0;      % tilt direction (deg)
    T_circle = 15;     % time for one full circle (s)
    N_circle = 150;    % points on the circle
    T_move   = 4;      % time zero -> circle, and back (s)
    dt       = T_circle / N_circle;

    % -180..+180 deg so joint 1 sweeps once without jumping
    a = linspace(-pi + 0.02, pi - 0.02, N_circle);
    Tc = zeros(4, 4, N_circle);
    for k = 1:N_circle
        Tc(:,:,k) = ceiling_to_base(disc_r*cos(a(k)), disc_r*sin(a(k)), ...
                                    CEILING, CLEARANCE, tilt_mag, tilt_phi);
    end

    fprintf('Calculating inverse kinematics...\n');
    Q = zeros(N_circle, 5);
    seed = [a(1), 0, 0, 0, 0];
    for k = 1:N_circle
        Q(k,:) = ik5(robot, Tc(1:3,4,k), Tc(1:3,3,k), seed);
        seed = Q(k,:);
    end
    Q = unwrap(Q);
    if any(abs(Q(:)) > pi)
        warning('Some joint angles are outside +-180 deg; Gazebo will clip them.');
    end

    figure; hold on; grid on; axis equal;
    edge = squeeze(Tc(1:3,4,:));
    fill3(edge(1,:), edge(2,:), edge(3,:), [0.8 0.8 0.8], 'FaceAlpha', 0.4);
    plot3(edge(1,:), edge(2,:), edge(3,:), 'r-', 'LineWidth', 1.5);
    robot.plot(q_zero);

    Nm = round(T_move / 0.1);
    fprintf('Moving to zero...\n');
    send_and_plot(robot, pub, msg, q_zero, 1.0);
    fprintf('Moving to circle start...\n');
    send_and_plot(robot, pub, msg, jtraj(q_zero, Q(1,:), Nm), 0.1);
    fprintf('Following the circle...\n');
    send_and_plot(robot, pub, msg, Q, dt);
    fprintf('Returning to zero...\n');
    send_and_plot(robot, pub, msg, jtraj(Q(end,:), q_zero, Nm), 0.1);
    fprintf('Circle done.\n');
end

%% ---------- Mode: drill (was motion_sim_gazebo.m) ----------
function run_drill(robot, pub, msg)
    [CEILING, CLEARANCE, DRILL_STROKE, DRILL_SPEED_UP] = load_config();
    q_zero = zeros(1,5);

    % Holes to drill (ceiling coordinates)
    points = [0.1,  0.2;
              0.2, -0.3;
              0,    0.5];
    tilt_mag = 15;
    tilt_phi = 0;

    dt = 0.1;                 % time step and Gazebo send interval (s)
    Tb = 3;                   % time per move between holes (s)
    Nb = round(Tb / dt);

    Vd   = DRILL_SPEED_UP / DRILL_STROKE;
    Tacc = 0.5;
    Td   = 1/Vd + Tacc;
    Nd   = round(Td / dt) + 1;
    td   = linspace(0, Td, Nd);

    sb = tpoly(0, 1, Nb);
    sd = lspb(0, 1, td, Vd);

    figure; hold on; grid on; axis equal;
    draw_disc(points, CEILING, tilt_mag, tilt_phi);
    robot.plot(q_zero);

    send_and_plot(robot, pub, msg, q_zero, 1.0);
    q_now = q_zero;

    for i = 1:size(points, 1)
        fprintf('Hole %d of %d\n', i, size(points, 1));
        P = ceiling_to_base(points(i,1), points(i,2), CEILING,  CLEARANCE,    tilt_mag, tilt_phi);
        D = ceiling_to_base(points(i,1), points(i,2), CEILING, -DRILL_STROKE, tilt_mag, tilt_phi);

        traj = between_traj(q_now, P, robot, sb, Nb);        % move below the hole
        send_and_plot(robot, pub, msg, traj, dt);  q_now = traj(end,:);

        traj = drill_traj(P, D, robot, sd, Nd, q_now);        % drill up
        send_and_plot(robot, pub, msg, traj, dt);  q_now = traj(end,:);

        traj = drill_traj(D, P, robot, sd, Nd, q_now);        % pull back down
        send_and_plot(robot, pub, msg, traj, dt);  q_now = traj(end,:);
    end

    fprintf('Returning to zero position\n');
    send_and_plot(robot, pub, msg, jtraj(q_now, q_zero, Nb), dt);
    fprintf('Motion finished.\n');
end

%% ---------- Ceiling fit (was fit_ceiling_to_tool.m) ----------
function fit_ceiling_to_tool(Q)
% Fits a plane through the Gazebo tool-tip positions on the circle and
% prints one sed command for the Ubuntu VM that moves the ceiling plate.
    Rz = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];
    Rx = @(t) [1 0 0 0; 0 cos(t) -sin(t) 0; 0 sin(t) cos(t) 0; 0 0 0 1];
    Tr = @(x,y,z) [eye(3) [x;y;z]; 0 0 0 1];
    h  = pi/2;
    fk = @(q) Rz(q(1))*Tr(0,0,0.15)*Rx(-h)*Rz(q(2))*Tr(0.6,0,0)*Rz(q(3))*Rx(-h) ...
            *Rz(q(4))*Tr(0,0,0.6)*Rx(h)*Rz(q(5))*Rx(-h)*Tr(0,0,0.35);

    N = size(Q, 1);
    tips = zeros(3, N);
    for k = 1:N
        T = fk(Q(k,:));
        tips(:,k) = T(1:3,4);
    end

    c = mean(tips, 2);
    [~, ~, V] = svd((tips - c)', 0);
    n = V(:,3);
    if n(3) < 0, n = -n; end
    fprintf('\nTip distance from fitted plane: max %.1f mm\n', 1000*max(abs(n'*(tips - c))));

    x = [1;0;0] - n*(n'*[1;0;0]);  x = x / norm(x);
    y = cross(n, x);
    R = [x y n];

    thickness = 0.01;
    p = c + n * thickness/2;

    [roll, pitch, yaw] = urdf_rpy(R);
    origin = sprintf('<origin xyz="%.4f %.4f %.4f" rpy="%.5f %.5f %.5f"/>', p, roll, pitch, yaw);

    fprintf('\nNew ceiling origin:\n  %s\n\n', origin);
    fprintf('Copy this ONE line into the Ubuntu terminal:\n\n');
    fprintf(['sed -i ''/name="base_to_ceiling"/,/<\\/joint>/ s|<origin .*/>|%s|'' ' ...
             '~/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro\n\n'], origin);
end

%% ---------- Mode: ceiling_urdf (was make_ceiling_urdf.m) ----------
function make_ceiling_urdf()
    CEILING = load_config();
    tilt_mag   = 10;    % deg (same as circle mode)
    tilt_phi   = 0;     % deg
    plate_size = 1.4;   % m

    T = double(ceiling_to_base(0, 0, CEILING, 0, tilt_mag, tilt_phi));
    [roll, pitch, yaw] = urdf_rpy(T(1:3,1:3));
    p = T(1:3,4);

    fprintf('\n<!-- ===== Ceiling plate (visual only, generated by robot_all_in_one.m) ===== -->\n');
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
end

%% ---------- Shared helpers ----------
function send_and_plot(robot, pub, msg, Q, dt)
% Send each row of Q to Gazebo and draw the same pose in MATLAB.
    for k = 1:size(Q, 1)
        q = Q(k,:);
        msg.data = q;
        send(pub, msg);
        robot.animate(q);
        drawnow;
        pause(dt);
    end
end

function draw_disc(points, CEILING, tilt_mag, tilt_phi)
% Grey 0.5 m disc on the ceiling, with any holes in "points" marked in red.
    a = linspace(0, 2*pi, 60);
    edge = zeros(3, numel(a));
    for k = 1:numel(a)
        Tp = ceiling_to_base(0.5*cos(a(k)), 0.5*sin(a(k)), CEILING, 0, tilt_mag, tilt_phi);
        edge(:,k) = Tp(1:3,4);
    end
    fill3(edge(1,:), edge(2,:), edge(3,:), [0.8 0.8 0.8], 'FaceAlpha', 0.4);
    for k = 1:size(points, 1)
        Tp = ceiling_to_base(points(k,1), points(k,2), CEILING, 0, tilt_mag, tilt_phi);
        plot3(Tp(1,4), Tp(2,4), Tp(3,4), 'ro', 'MarkerFaceColor', 'r');
    end
end

function traj = between_traj(q1, P2, robot, s, N)
% Rotate the base towards the next hole, then move the other joints to it.
    [phi2, ~] = cart2pol(P2(1,4), P2(2,4));
    base_traj = q1 + s * [phi2 - q1(1), 0, 0, 0, 0];
    q_last = base_traj(N,:);
    q2 = ik5(robot, P2(1:3,4), P2(1:3,3), q_last);
    traj = [base_traj; jtraj(q_last, q2, N)];
end

function traj = drill_traj(P, D, robot, s, N, seed)
% Straight-line drill motion, each IK step seeded with the previous one.
    Ttraj = ctraj(P, D, s);
    traj = zeros(N, 5);
    for i = 1:N
        traj(i,:) = ik5(robot, Ttraj(1:3,4,i), Ttraj(1:3,3,i), seed);
        seed = traj(i,:);
    end
end

function [roll, pitch, yaw] = urdf_rpy(R)
% URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    pitch = -asin(max(-1, min(1, R(3,1))));
    roll  = atan2(R(3,2), R(3,3));
    yaw   = atan2(R(2,1), R(1,1));
end
