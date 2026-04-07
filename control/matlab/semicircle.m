function [x_semi, y_semi, Cx, Cy, R] = semicircle(x1, y1, x2, y2, n_points)
% SEMICIRCLE Generates and plots a semicircle between two given points.
%            Automatically draws the trajectory on an existing workspace figure.
%
% Inputs:
%   x1, y1   - Coordinates of the first point
%   x2, y2   - Coordinates of the second point
%   n_points - Number of points to generate along the semicircle
%
% Outputs:
%   x_semi, y_semi - Coordinates of the semicircle points
%   Cx, Cy         - Center of the semicircle
%   R              - Radius of the semicircle

    % Validate number of points
    if n_points < 2
        error('n_points must be at least 2.');
    end

    % Calculate distance and radius
    d = sqrt((x2 - x1)^2 + (y2 - y1)^2);
    R = d / 2;

    % Calculate midpoint
    Mx = (x1 + x2) / 2;
    My = (y1 + y2) / 2;

    % Height from midpoint to center
    h = sqrt(R^2 - (d / 2)^2);

    % Unit vector perpendicular to the segment
    dx = x2 - x1;
    dy = y2 - y1;
    u_x = -dy / d;
    u_y =  dx / d;

    % Center of the semicircle
    Cx = Mx + h * u_x;
    Cy = My + h * u_y;

    % Generate the semicircle points
    theta = linspace(atan2(y1 - Cy, x1 - Cx), atan2(y2 - Cy, x2 - Cx), n_points);
    x_semi = Cx + R * cos(theta);
    y_semi = Cy + R * sin(theta);

    % --- Draw on existing figure (workspace) or create a new one ---
    if ~isempty(get(0, 'CurrentFigure')) && ishold(gca)
        % A figure with hold on exists: draw on top of it
        plot(x_semi, y_semi, 'r-',  'LineWidth', 2);
        plot([x1, x2], [y1, y2],   'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot(Cx, Cy,                'kx', 'MarkerSize', 8, 'LineWidth', 2);
    else
        % No active figure: create a standalone plot
        figure;
        plot(x_semi, y_semi, 'g-',  'LineWidth', 2);
        hold on;
        plot([x1, x2], [y1, y2],   'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        plot(Cx, Cy,                'kx', 'MarkerSize', 8, 'LineWidth', 2);
        title('Semicircle');
        xlabel('X'); ylabel('Y');
        axis equal; grid on;
    end

    % Legend
    legend('Workspace', 'Semicircle', 'Endpoints', 'Center', ...
           'Location', 'best');

end