load robot_model_data.mat;

ceiling_height = 1.15;
clearance = 0.15;
safe_plane = ceiling_height - clearance;

% Points (ceiling frame)
point_ceiling = [0.0, -0.5];

% Drill points (base frame)
pstart = [0, 0.5, safe_plane];
pend = [point_ceiling, safe_plane];

% Starting pose and joint space

Tc = robot.fkine(qz); %Current pose


Tstart = transl(pstart); %Next pose 
Tend = transl(pend);


qend = robot.ikine(Tend, 'mask', [1, 1, 1, 1, 1, 0]);
qstart = robot.ikine(Tstart, 'mask', [1, 1, 1, 1, 1, 0]); %Next joint space



%main
T = 3; %time [s]
dt = 0.1; %time between steps [s]
ts = T/dt; %time steps




time = tic;

car_traj = ctraj(Tstart, Tend, ts)
joint_traj = jtraj(qstart, qend, ts);


n = 1;
q = qstart;
q(1) = atan2(point_ceiling(2), point_ceiling(1));

% robot.plot(qstart);

while n <= ts
    q = robot.ikine(car_traj(:, :, n), q, 'mask', [1, 1, 1, 1, 1, 0])
    robot.plot(q);
    %robot.plot(joint_trajectory(n, :));
    n = n + 1;
end








%base tilt



% * r2t(t2r(Tc)




% while run
%     if toc(time) >= dt
%         q = q + [0.05, 0, 0, 0, 0];
%         robot.plot(q)
% 
%         time = tic;
%     end
% 
% end








% Trajectories

%q0 -> qi1
%qi1 -> qp1
%qp1 -> qi1
%qi1 -> qi2
%qi2 -> qp2
%qp2 -> qi2
%qi2 -> q0


%T_actual = robot.fkine(qstart) %test