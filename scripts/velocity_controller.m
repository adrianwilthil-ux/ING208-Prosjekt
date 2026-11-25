addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
joint_dynamics;

%Velocity controller
Kp = 2.5; %Proportional gain
Ki = 1; %Integral gain

Cv = pid(Kp, Ki);
tf_vel_controller = feedback(Cv*tf_joint, 1);

%Plot step response
figure;
step(tf_vel_controller);
grid on;

title('Joint 2 - Velocity controller');
ylabel('Joint velocity (rad/s)');
xlabel('Time (s)');