%% ELE208 Overhead Drilling Robot - Differential Kinematics Simulation
clear; clc; close all;
% 1) 2R planar :
fprintf(' 2R Planar Analogue \n');
syms theta1 theta2 a1_sym a2_sym real  %variables

T_02 = [cos(theta1+theta2) -sin(theta1+theta2) 0 a1_sym*cos(theta1)+a2_sym*cos(theta1+theta2);
        sin(theta1+theta2)  cos(theta1+theta2) 0 a1_sym*sin(theta1)+a2_sym*sin(theta1+theta2);
        0 0 1 0;
        0 0 0 1];                                  % formula 
Tp_sym = T_02(1:2,4);              %
J_sym = jacobian(Tp_sym, [theta1 theta2]);

% Numeric values for drilling test
a1_val = 0.6; a2_val = 0.6; % arm length 
theta1_val = deg2rad(167.028);
theta2_val = deg2rad(-156.44);

J_num = double(subs(J_sym, [theta1, theta2, a1_sym, a2_sym], ...
                          [theta1_val, theta2_val, a1_val, a2_val])); % real numbers into the formula
                      
xdot_des = [0; 0.10]; % move 10cm/sec
qdot_drill = J_num \ xdot_des; % reverse if 10cm/sec then what speed?
fprintf('Required joint speeds for 2R planar (rad/s): q1dot = %.3f, q2dot = %.3f\n\n', qdot_drill(1), qdot_drill(2));



% 2) Full 5-DOF Robot Simulation (Resolved-Rate Control)
fprintf('5-DOF Robot Simulation\n');

% DH Parameters:
L_B = 0.15; L = 0.60; L_tool = 0.35 ; % base length, arm length , drill tool length (0.15+0.2) 

L0 = Link('d', L_B, 'a', 0, 'alpha', -pi/2);
L1 = Link('d', 0,   'a', L, 'alpha',  0);
L2 = Link('d', 0,   'a', 0, 'alpha', -pi/2);
L3 = Link('d', L,   'a', 0, 'alpha',  pi/2);
L4 = Link('d', 0,   'a', 0, 'alpha', -pi/2);

robot = SerialLink([L0 L1 L2 L3 L4], 'name', 'DrillBot'); % putting the 5 joints together to make the DrillBot
robot.tool = transl(0, 0, L_tool); % joining End- Effector to the wrist.

robot.qlim = deg2rad([-180 180; -90 90; 70 160; -180 180; -90 90]);% safety margin 

Z_roof = 1.00; drill_depth = 0.20; v_drill = 0.10; dt = 0.01;   %  Dynamics and control requirements  in mm
t_drill = drill_depth / v_drill; N_drill = round(t_drill/dt);  % 20/10= 2sec, 2/0.01=200 tiny steps to drill 

targets = [ 0.00  0.00; 0.50  0.00; 0.35  0.35];  
target_names = {'Center', 'Edge (+x)', 'Diagonal'};

q_guess = deg2rad([0, 30, 120, -60, 0]); 
figure('Name', '3D Drilling Paths', 'Color', 'w');

for k = 1:size(targets,1)
     z_tool = [0; 0; 1];% drilling 3 holes (center, far right , diagonal)
    x_tool = [1; 0; 0]; 
    y_tool = cross(z_tool, x_tool);
   
    R_target = [x_tool, y_tool, z_tool]; % pointing drill straight up st the roof
    
    p_start = [targets(k,1), targets(k,2), Z_roof - drill_depth];% 20 cm below 
    T_start = [R_target, p_start'; 0 0 0 1];
    
    q_start = robot.ikine(T_start, 'q0', q_guess, 'mask', [1 1 1 0 0 0]);
    if isempty(q_start), error('IK failed at start position.'); 
    end
    q_guess = q_start; 
    
    q = q_start;
    Q = zeros(N_drill, 5);
    
    for i = 1:N_drill
        v_des = [0; 0; v_drill];
        J = robot.jacob0(q);
        Jv = J(1:3, :);
        dq = pinv(Jv) * v_des;
        
        q = q + (dq' * dt);
        q = max(min(q, robot.qlim(:,2)'), robot.qlim(:,1)');
        Q(i,:) = q;
    end
    
    % EXTRACTION CODE 
    T_all = robot.fkine(Q);
    P = zeros(N_drill, 3);
    try
        
        for i = 1:N_drill
            P(i,:) = T_all(i).t;
        end
    catch
       
    end
    
    % plotting:
    subplot(1, 3, k);
    robot.plot(Q(1:10:end,:), 'delay', 0, 'workspace', [-1 1 -1 1 0 1.5], 'scale', 0.5);
    hold on;
    plot3(P(:,1), P(:,2), P(:,3), 'r-', 'LineWidth', 2);
    plot3(targets(k,1), targets(k,2), Z_roof, 'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    title(sprintf('Drill: %s', target_names{k}));
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(45, 30); grid on;
end