% Importing robot model
load robot_model_data.mat

% Symbolic angles and test angles
syms theta_b theta_s theta_e theta_w1 theta_w2;
thetas = [theta_b theta_s theta_e theta_w1 theta_w2];
q1 = [0, 0, 0, 0, 0];
q2 = [0.1, 0.2, 0.3, 0.4, 0.5];
q3 = [0.2, 0.1, 0.4, 0.1, 0.2];


%% Manual forward kinematics

% Defining DH to Transformation matrix 
function A = dh(theta, d, a, alpha)
A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0,           sin(alpha),             cos(alpha),            d;
     0,           0,                       0,                    1];
end

% Joint frames per DH row + tool transformation
Tb = dh(theta_b, lb, 0, -sym(pi)/2);
Ts = dh(theta_s, 0, L, 0);
Te = dh(theta_e, 0, 0, -sym(pi)/2);
Tw1 = dh(theta_w1, L, 0, sym(pi)/2);
Tw2 = dh(theta_w2, 0, 0, -sym(pi)/2);
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
disp('--- TEST q1 ---');
manual = double(subs(T, thetas, q1))
corke = robot.fkine(q1)

disp('--- TEST q2 ---');
manual = double(subs(T, thetas, q2))
corke = robot.fkine(q2)

disp('--- TEST q3 ---');
manual = double(subs(T, thetas, q3))
corke = robot.fkine(q3)