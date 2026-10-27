d = 1; %arm base to roof along robot z-axis
tl = 0.15; %tool length (fully drilled)
ll = 0.6; %shoulder and elbow link length

N = d * cosd(15); %arm base to roof normal

%% Maximum extension
Tlmax = sqrt((0.5 + d * sind(15))^2 + (N - tl)^2); %arm base to tool base max distance
thetamax = (90 - acosd(Tlmax/2 / ll)) * 2

%%Minimum extension
Tlmin = N - tl; %arm base to tool base min distance (straight above tool base)
thetamin = (90 - acosd(Tlmin/2 / ll)) * 2