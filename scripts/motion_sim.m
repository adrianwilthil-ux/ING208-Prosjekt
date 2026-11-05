load robot_model_data.mat;

ceiling_height = 1.15;
clearance = 0.55;
safe_plane = ceiling_height - clearance;

% Points (ceiling frame)
point_ceiling1 = [0.3, -0.3];
point_ceiling2 = [-0.3, 0.3];

% Drill points and poses (base frame)
pstart = [point_ceiling1, safe_plane];
pend = [point_ceiling2, safe_plane];

Tstart = transl(pstart) * r2t(t2r(robot.fkine(qz))); 
Tend = transl(pend);

qstart = robot.ikine(Tstart, 'mask', [1, 1, 1, 1, 1, 0]); 
qend = robot.ikine(Tend, 'mask', [1, 1, 1, 1, 1, 0]);



%main
T = 3; %time [s]
dt = 0.1; %time between steps [s]
ts = 30; %time steps




time = tic;

cart_traj = ctraj(Tstart, Tend, ts);
joint_traj = jtraj(qstart, qend, ts);


robot.ikine(cart_traj(:, :, 19), 'mask', [1, 1, 1, 1, 1, 0])




n = 1;
q = qstart;
q(1) = atan2(point_ceiling1(2), point_ceiling1(1));

robot.plot(qstart);

while n <= ts
    q = robot.ikine(cart_traj(:, :, n), q, 'mask', [1, 1, 1, 1, 1, 0]);
    robot.plot(q);
    %robot.plot(joint_traj(n, :));
    n = n + 1
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