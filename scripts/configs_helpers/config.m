%Workspace
CEILING = 1.15; %m
CLEARANCE = 0.2; %m
DISC_RADIUS = 0.5; %m

%Drilling params
DRILL_STROKE = 0.2; %m
DRILL_SPEED_UP = 0.01; %m/s

%DH values
BASE_HEIGHT = 0.15; %m
LINK_LENGTH = 0.6; %m

CASING_LENGTH = 0.2;
BIT_LENGTH = 0.25;
TOOL_LENGTH = CASING_LENGTH / 2 + BIT_LENGTH; %m

%Dynamics values
DENSITY = 2700; %kg/m^3 (aluminium)
LENGTHS = [BASE_HEIGHT, LINK_LENGTH, LINK_LENGTH/2, LINK_LENGTH/2];
CROSS_SECTIONS = [0.13, 0.08, 0.06, 0.06]; %m
COMS = [0,            LENGTHS(1)/2, 0;
       -LENGTHS(2)/2, 0,            0;
        0,            0,            LENGTHS(3)/2;
        0,           -LENGTHS(4)/2, 0;           ];

DRILL_CASING_MASS = 3;
DRILL_BIT_MASS = 0.13;
DRILL_COM = [0, 0, 0.225*DRILL_BIT_MASS/(DRILL_BIT_MASS + DRILL_CASING_MASS)];

DRILL_FORCE = 50; %N