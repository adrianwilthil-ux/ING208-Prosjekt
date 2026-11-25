%% IK5 - Custom inverse kinematic solver (claude assisted)

function q = ik5(robot, p, d, qprev)
% Closed-form IK: tip position p (1x3) and tool z axis d (1x3), both in base frame.
% Returns the solution closest to qprev, or [] if unreachable.

% Get params from robot object (dont wanna run config file within a function)
O  = robot.offset;  lb = robot.d(1);  L = robot.a(2); 
Tt = double(robot.tool);  tl = Tt(3,4); % Tool length

d  = d(:)/norm(d);  p = p(:);% Column vectors, unit axis
w  = p - tl*d - [0;0;lb]; % Wrist point relative to the shoulder
wrap = @(a) atan2(sin(a), cos(a)); % Wrap angles into (-pi, pi]
best = inf;  q = []; %Initializing values

%% -- AI Written function -- (Details in App. C)
for flip = 0:1 % Base branch: facing the point, or reaching over

    if hypot(w(1), w(2)) < 1e-9, q1 = qprev(1) + flip*pi; % On the axis: keep the previous base angle
    else, q1 = atan2(w(2), w(1)) + flip*pi; end
    rho = w(1)*cos(q1) + w(2)*sin(q1);  h = w(3); % Signed radius and height in the arm plane
    c = (rho^2 + h^2)/(2*L^2) - 1; % Cos of the elbow angle (law of cosines)
    if abs(c) > 1, continue; end % Out of reach on this branch
    
    for el = [-1 1] % Elbow up / down
        beta = el*acos(c);  th3 = -beta - pi/2;  th2 = beta/2 - atan2(h, rho);
        r  = [cos(q1); sin(q1); 0];  z = [0; 0; 1]; % Plane directions in the world
        x2 = cos(th2)*r - sin(th2)*z;  y2 = -sin(th2)*r - cos(th2)*z;
        x3 = cos(th3)*x2 + sin(th3)*y2;  z3 = -sin(th3)*x2 + cos(th3)*y2;
        y3 = [sin(q1); -cos(q1); 0]; % Wrist frame axes
        c5 = max(-1, min(1, d'*z3)); % Tool axis vs z3 gives joint 5

        for wr = [-1 1] % Wrist flip
            th5 = wr*acos(c5);

            if abs(sin(th5)) < 1e-9, th4 = qprev(4) + O(4); % Wrist singularity: keep q4
            else, sg = sign(sin(th5)); th4 = atan2(-sg*(d'*y3), -sg*(d'*x3)); end
            cand = [q1, th2-O(2), th3-O(3), th4-O(4), th5-O(5)]; % Undo the DH offsets
            cost = sum(abs(wrap(cand - qprev))); % Distance to the previous solution
            
            if cost < best, best = cost; q = cand; end
        end
    end
end
end