%Lengths
lb = 0.15;
L = 0.6;
tl = 0.35;
T_tool = transl(0, 0, tl);

% Robot object using DH params
Links(1) = Link('d', lb, 'a', 0,  'alpha',  -pi/2);
Links(2) = Link('d', 0,  'a', L, 'alpha',  0);
Links(3) = Link('d', 0,  'a', 0, 'alpha',  -pi/2);
Links(4) = Link('d', L,  'a', 0,  'alpha',  pi/2);
Links(5) = Link('d', 0,  'a', 0,  'alpha',  -pi/2);
T_tool = transl(0, 0, tl);

robot = SerialLink(Links, 'name', 'myRobot', 'tool', T_tool);

% Robot object for symbolic matrix
Links_sym(1) = Link('d', lb, 'a', 0,  'alpha',  sym(-pi/2));
Links_sym(2) = Link('d', 0,  'a', L, 'alpha',  0);
Links_sym(3) = Link('d', 0,  'a', 0, 'alpha',  sym(-pi/2));
Links_sym(4) = Link('d', L,  'a', 0,  'alpha',  sym(pi/2));
Links_sym(5) = Link('d', 0,  'a', 0,  'alpha',  sym(-pi/2));
T_tool = transl(0, 0, tl);

robot_sym = SerialLink(Links_sym, 'name', 'myRobot', 'tool', T_tool);

save('robot_model.mat', 'lb', 'L', 'tl', 'robot', 'robot_sym');