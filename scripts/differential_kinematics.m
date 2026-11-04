load robot_model_data.mat

syms q1 q2 q3 q4 q5;
q = [q1, q2, q3, q4, q5];
qdot = [0.1, 0.1, 0.1, 0.1, 0.1]';

J = robot.jacobe(q);
J = J(1: 5, :);

det_J = simplify(det(J));
singularities = solve(det_J == 0, q)