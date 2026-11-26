%% Singularities - checks rank of singular poses

% Importing robot model
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
robot_model;

Q = deg2rad([0    0      0   0    0;    % Regular pose
             0  -22.5    0   0    0;    % Wrist centre on base axis
             0    0    135   0    0;    % Straight elbow
             0    0      0   0  -45]);  % Drill aligned with twist axis

for i = 1:4
    J = robot.jacobe(Q(i,:));
    fprintf('Pose %d: rank = %d\n', i, rank(J(1:5,:)));
end