% Importing robot model
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
robot_model;

% Symbolic angles and test angles
syms qb qs qe qw1 qw2;
thetas = [qb qs qe qw1 qw2];
Q1 = [0, 0, 0, 0, 0];
Q2 = [0.1, 0.2, 0.3, 0.4, 0.5];
Q3 = [0.2, 0.1, 0.4, 0.1, 0.2];


%% Manual forward kinematics

% Defining DH to Transformation matrix 
function A = dh(theta, d, a, alpha)
A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0,           sin(alpha),             cos(alpha),            d;
     0,           0,                       0,                    1];
end

% Joint frames per DH row + tool transformation
Tb = dh(qb, lb, 0, -sym(pi)/2);
Ts = dh(qs, 0, L, 0);
Te = dh(qe, 0, 0, -sym(pi)/2);
Tw1 = dh(qw1, L, 0, sym(pi)/2);
Tw2 = dh(qw2, 0, 0, -sym(pi)/2);
Tt = [1 0 0 0;
      0 1 0 0;
      0 0 1 tl;
      0 0 0 1];

% Final transformation (base to tooltip)
T =  Tb * Ts * Te * Tw1 * Tw2 * Tt;



%% Testing for equivalance

% General equivalance (SYMBOLIC ANGLES)
disp('--- GENERAL MATRIX ---');
disp('manual: ');
disp(T);
disp('corke:');
robot_sym.fkine(thetas)


% Numerical equivalanve (GIVEN ANGLES)
disp('--- TEST Q1 ---');
manual = double(subs(T, thetas, Q1))
corke = robot.fkine(Q1)

disp('--- TEST Q2 ---');
manual = double(subs(T, thetas, Q2))
corke = robot.fkine(Q2)

disp('--- TEST Q3 ---');
manual = double(subs(T, thetas, Q3))
corke = robot.fkine(Q3)