load robot_model_data.mat

q = [0, 0, 0, 0, 0];
qdot = [0.1, 0.1, 0.1, 0.1, 0.1]';

robot.jacob0(q)*qdot