%% Position controller
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
velocity_controller;

%P controller
Kp = 1.5; %Proportional gain
Cp = pid(Kp);

%Start and end angles
q_start = 0;
q_end = 1;


%Transfer function
tf_pos = tf_vel_controller/s; %Integrating velocity TF to get position
tf_pos_controller = feedback(Cp*tf_pos, 1); %Closed loop

%Plotting (Written by AI)
figure;
step((q_end-q_start)*tf_pos_controller);
grid on;
title('Joint 2 - Position controller');
xlabel('Time (s)');
ylabel('Joint position (rad)');

info = stepinfo(tf_pos_controller)
error_deg = abs(1 - dcgain(tf_pos_controller)) * 180/pi


