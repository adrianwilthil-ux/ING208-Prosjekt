%% PLOT DISC

function plot_disc(points, ceiling, mag, phi)

    clf;
    disc_r = 0.5;                                   % disc radius (m)
    a = linspace(0, 2*pi, 60);                      % angles around the edge
    edge = zeros(3, numel(a));                      % edge points in the base frame
    for i = 1:numel(a)
        Tp = ceiling_to_base(disc_r*cos(a(i)), disc_r*sin(a(i)), ceiling, 0, mag, phi); % clearance 0 = the ceiling surface itself
        edge(:, i) = Tp(1:3, 4);                    % keep only the position
    end
    hold on;                                        % keep the robot in the figure
    fill3(edge(1,:), edge(2,:), edge(3,:), [0.8 0.8 0.8], 'FaceAlpha', 0.4); % filled disc
   
    for i = 1:size(points, 1)
        T = ceiling_to_base(points(i, 1), points(i, 2), ceiling, 0, mag, phi);
        plot3(T(1,4), T(2,4), T(3,4), 'bo', 'MarkerFaceColor', 'b'); % start tip position
    end
end