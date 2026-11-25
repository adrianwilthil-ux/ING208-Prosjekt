%% Sensor transform - transforms camera frame point and tilt data to pose in base frame

addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

p = [0, 0, 1]'; % Point spotted by cameras [m]
tilt_mag = 15; % Magnitude of tilt [deg]
tilt_phi = 35; % Direction of tilt [deg]

p_h = [p; 1]; % Homogeneous point

T = transl(0, 0, 0.15); % Camera transformation
R = rotz(deg2rad(tilt_phi)) * roty(deg2rad(tilt_mag)); % Base tilt

p_b = T*p_h; % Transformation of point to base frame
p_b = p_b(1:3); % Back to non-homogeneous

P_b = [R, p_b;   % Pose in base frame
       0 0 0 1];

q = ik5(robot, p_b, R(:, 3), qz); % Inverse kinematics

plot_disc([p0, p0], CEILING, tilt_mag, tilt_phi);
robot.plot(q)


disp("Point in camera frame");
disp(p');
disp("Tilt magnitude: " + tilt_mag + "  Tilt direction: " + tilt_phi + newline);
disp("Pose in base frame:");
disp(P_b);
disp("Joint angles:");
disp(q);