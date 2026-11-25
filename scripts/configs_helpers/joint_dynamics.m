config;
robot_model;

% Joint 2 inertia
M = robot.inertia(qz); %Inertia at 0-pose
Jl = M(2,2);

Jm = 0.000201; %Motor moment of inertia [kg*m^2]
G = 81; %Gearbox ratio
Km = 0.11; %Torque constant [Nm/A]
Bm = 0;

% Effective inertia at motor shaft
J_eff = Jm + Jl/G^2
B_eff = Bm; % Neglecting friction

% Transfer function: motor current to motor speed
s = tf('s'); % TF variable
tf_motor = Km/(J_eff*s + B_eff); % Motor ransfer function
tf_joint = tf_motor/G; % Joint transfer function (with gearbox)

disp("Shoulder transfer function: ");
tf_joint
