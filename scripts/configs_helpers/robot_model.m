config;

%Lengths and offsets
lb = BASE_HEIGHT;
L = LINK_LENGTH;
tl = TOOL_LENGTH;
O = [0, 0, -5*pi/4, 0, pi/4]; %Offsets

% Robot object using DH params
Links(1) = Link('d', lb, 'a', 0,  'alpha',  -pi/2, 'offset', O(1));
Links(2) = Link('d', 0,  'a', L, 'alpha',  0, 'offset', O(2));
Links(3) = Link('d', 0,  'a', 0, 'alpha',  -pi/2, 'offset', O(3));
Links(4) = Link('d', L,  'a', 0,  'alpha',  pi/2, 'offset', O(4));
Links(5) = Link('d', 0,  'a', 0,  'alpha',  -pi/2, 'offset', O(5));
T_tool = transl(0, 0, tl);

robot = SerialLink(Links, 'name', 'myRobot', 'tool', T_tool);

%Misc
qz = [0, 0, 0, 0, 0];
mask = [1, 1, 1, 1, 1, 0];




% Robot object for symbolic matrix
Links_sym(1) = Link('d', lb, 'a', 0,  'alpha',  sym(-pi/2));
Links_sym(2) = Link('d', 0,  'a', L, 'alpha',  0);
Links_sym(3) = Link('d', 0,  'a', 0, 'alpha',  sym(-pi/2));
Links_sym(4) = Link('d', L,  'a', 0,  'alpha',  sym(pi/2));
Links_sym(5) = Link('d', 0,  'a', 0,  'alpha',  sym(-pi/2));
T_tool = transl(0, 0, tl);

robot_sym = SerialLink(Links_sym, 'name', 'myRobot', 'tool', T_tool);