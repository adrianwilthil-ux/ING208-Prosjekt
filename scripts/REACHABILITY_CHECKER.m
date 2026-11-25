%% Reachability checker - tests all required poses

%Imports
addpath(fullfile(fileparts(mfilename('fullpath')), 'configs_helpers'));
config;
robot_model;

%Resolution
res_h = 4; %Resolution height [cm]
res_p = 2; %Resolution points [cm]
res_t = 3; %Resolution tilt [deg]

%Start and end height offsets
start_offset = -DRILL_STROKE * 100 / res_h; %cm / res
end_offset = CLEARANCE * 100 / res_h; %cm / res

%Start and end tilt
start_mag = 0 / res_t;
end_mag = 15 / res_t;

%Scaled radius
radius_scaled = DISC_RADIUS * 100 / res_p; %cm / res

%Initializing variables
reachable = 0;
unreachable = 0;

peak_qd = zeros(5,1); 
peak_q = nan(5,5); 
peak_trq = zeros(5,1);
peak_trq_q = nan(5,5); 

failed_speed = 0; 
v = [0; 0; 0.10; 0; 0]; %Target speed
qprev = qz;

%Main test loop
for h = start_offset:end_offset    %Every height
    height_real = h * res_h / 100;
    for m = start_mag:end_mag      %Every tilt angle
        mag_real = m * res_t;
        
        for x = -radius_scaled:radius_scaled   %Every x-step in disc
            x_real = x * res_p / 100;
            y_steps = floor(sqrt(radius_scaled^2 - (x)^2));
            
            for y = -y_steps:y_steps           %Every y-step in disc
                y_real = y * res_p / 100;         
                Tp = ceiling_to_base(x_real, y_real, CEILING, height_real, mag_real, 0); %Transform point to robot frame
                q = ik5(robot, Tp(1:3, 4), Tp(1:3, 3), qprev); %Inverse kinematics, get joint angles
                qprev = q; %Seed for next iteration
                
                if isempty(q) %If pose unreachable, count
                    unreachable = unreachable + 1;
                    %disp(["X:", x_real, "Y:", y_real, "Offset:", height_real, "Tilt:", mag_real]); %For troubleshooting

                else %If reachable, count
                    reachable = reachable + 1;

                    %10cm/s test
                    J = robot.jacobe(q); %Jacobian for every reachable pose
                    Jtask = J(1:5,:); %Dropping rotation around z as that is free
                    
                    if rcond(Jtask) < 1e-10 %If nearly singular matrix
                        qd = pinv(Jtask, 1e-10) * v; %Get joint speeds from psuedoinverse jacobian
                    else
                        qd = inv(Jtask) * v; %Else solve directly with inverse jacobian
                    end
                    
                    err = Jtask * qd - v; %Velocity error
                    if norm(err(1:3)) < 1e-6 && norm(err(4:5)) < 1e-6 %If both linear and angular error within tolerance
                        for i = 1:5 %For every joint
                            if abs(qd(i)) > peak_qd(i) %If speed larger than peak speed
                                peak_qd(i) = abs(qd(i)); %Store peak speed
                                peak_q(i,:) = q; %Store joint position
                            end
                        end
                    else
                        failed_speed = failed_speed + 1; %Failed speed count +1
                    end
                    
                end
            end
        end
    end
end

%Outputs
percentage = reachable / (unreachable + reachable) * 100;

disp("Reachable poses:");
disp(reachable);
disp("Unreachable poses:");
disp(unreachable);
disp("Percentage of poses reachable:");
disp([num2str(percentage), '%']);
disp(newline + "Poses with speed criteria failed: ")
disp(failed_speed);
disp("Peak joint speed:");
disp(peak_qd);
robot.plot(peak_q(1, :));