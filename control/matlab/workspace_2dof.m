function [lx, ly] = workspace_2dof(r1, r2, ang_range1, ang_range2, plot_result)
% WORKSPACE_2DOF Computes the reachable workspace of a 2-DOF SCARA.
%
% Inputs:
%   r1          - Length of link 1 [m]
%   r2          - Length of link 2 [m]
%   ang_range1  - Angle range for joint 1 [deg], e.g. -90:1:90
%   ang_range2  - Angle range for joint 2 [deg], e.g. -90:1:90
%   plot_result - (optional) true/false to display the workspace plot
%                 Default: false
%
% Outputs:
%   lx - X coordinates of the end-effector [m]
%   ly - Y coordinates of the end-effector [m]

    % Default value for plot_result
    if nargin < 5
        plot_result = false;
    end

    % Validate link lengths
    if r1 <= 0 || r2 <= 0
        error('Link lengths r1 and r2 must be positive.');
    end

    % Create grid of angle combinations
    [grid_brazo1, grid_brazo2] = meshgrid(ang_range1, ang_range2);

    % Flatten to column vectors and convert to radians
    th1 = deg2rad(grid_brazo1(:));
    th2 = deg2rad(grid_brazo2(:));

    % Forward kinematics: end-effector position
    ly = r1*cos(th1) + r2*cos(th1 + th2);
    lx = r1*sin(th1) + r2*sin(th1 + th2);

    % Optional plot
    if plot_result
        figure;
        plot(lx, ly, 'b.', 'MarkerSize', 2);
        title('End-Effector Trajectory');
        xlabel('X Position [m]');
        ylabel('Y Position [m]');
        ylim([-(r1+r2), (r1+r2)]);
        axis equal;
        grid on;
        hold on;
    end

end