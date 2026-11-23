addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

res_h = 1; %resolution height
res_p = 1; %resolution points
res_t = 1; %resolution tilt

start_offset = -DRILL_STROKE * 100 / res_h; %cm / res
end_offset = CLEARANCE * 100 / res_h; %cm / res

start_mag = 0 / res_t;
end_mag = 15 / res_t;

radius_scaled = DISC_RADIUS * 100 / res_p;

reachable = 0;
unreachable = 0;
peak_qd = zeros(5,1);


for h = start_offset:end_offset    %every height
    height_real = h * res_h / 100;
    for m = start_mag:end_mag      %every tilt angle
        mag_real = m * res_t;
        
        for x = -radius_scaled:radius_scaled   %every x-step
            x_real = x * res_p / 100;
            y_steps = floor(sqrt(radius_scaled^2 - (x)^2));
            
            for y = -y_steps:y_steps           %every y-step
                y_real = y * res_p / 100;         
                Tp = ceiling_to_base(x_real, y_real, CEILING, height_real, mag_real, 0); %point frame
                q = ik5(robot, Tp(1:3, 4), Tp(1:3, 3), qz);
                
                if isempty(q)
                    unreachable = unreachable + 1;
                    disp(["X:", x_real, "Y:", y_real, "Offset:", height_real, "Tilt:", mag_real]);
                else
                    reachable = reachable + 1;
                    J = robot.jacobe(q);
                    Jtask = J(1:5, :);                 % Keep drill direction fixed; tool roll is free
                    qd = Jtask \ [0; 0; 0.10; 0; 0];   % Required joint speeds [rad/s]
                    peak_qd = max(peak_qd, abs(qd));
                    %disp(["X:", x_real, "Y:", y_real, "Offset:", height_real, "Tilt:", mag_real]);
                end
            end
        end
    end
end

percentage = reachable / (unreachable + reachable) * 100;

disp("Reachable poses:");
disp(reachable);
disp("Unreachable poses:");
disp(unreachable);
disp("Percentage of poses reachable:");
disp([num2str(percentage), '%']);
disp("Peak joint speed:");
disp(peak_qd);