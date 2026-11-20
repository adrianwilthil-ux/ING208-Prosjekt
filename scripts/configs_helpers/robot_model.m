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

% Dynamic parameters
length_axis = [2, 1, 3, 2];

for i = 1:4
    l = LENGTHS(i);
    b = CROSS_SECTIONS(i);
    m = DENSITY * l * b^2;
    
    I_parallel = m * b^2 / 6;
    I_perp = m * (l^2 + b^2) / 12;
    
    Ixyz = [I_perp, I_perp, I_perp];
    Ixyz(length_axis(i)) = I_parallel;


    Links(i).m = m; %Mass
    Links(i).r = COMS(i, :);
    Links(i).I = diag(Ixyz);
end

Links(5).m = DRILL_CASING_MASS + DRILL_BIT_MASS;
Links(5).r = DRILL_COM;

z = DRILL_COM(3);

I_perp = DRILL_CASING_MASS * (CASING_LENGTH^2/12 + z^2) ...
       + DRILL_BIT_MASS * (BIT_LENGTH^2/12 + (0.225 - z)^2);
Links(5).I = diag([I_perp, I_perp, 0]);


robot = SerialLink(Links, 'name', 'myRobot', 'tool', T_tool);

% Gravity
robot.gravity = [0, 0, 9.81];


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



