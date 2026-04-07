clc
close all
clear
%% Dimensions and constrains
% Define the dimensions of the SCARA robot
base.r = 0.5; % [m]
base.l = 1.0; % [m]

l1.x = 1.0; % [m]
l1.y = 0.2; % [m]
l1.z = 0.2; % [m]

l2.x = 1.0; % [m]
l2.y = 0.2; % [m]
l2.z = 0.2; % [m]

man.x = 0.2; % [m]
man.y = 0.2; % [m]
man.z = 0.2; % [m]

% Define the joints constrains
l1.theta_min = -90; % degrees
l1.theta_max = 90; % degrees

l2.theta_min = -90; % degrees
l2.theta_max = 90; % degrees
%% Workspace
[lx, ly] = workspace_2dof(l1.x, l2.x, l1.theta_min:1:l1.theta_max, l2.theta_min:1:l2.theta_max, true);
%% Waypoints
n = 20; % # Waypoints
x1 = -0.5;
y1 = 1.5;
x2 = 0.5;
y2 = 1.5;
% [x_semi, y_semi, Cx, Cy, R] = semicircle(x1,y1,x2,y2,n);
% wp(1,:) = x_semi; % X
% wp(2,:) = y_semi; % Y
% wp(3,:) = ones([1,n])*(base.l + l1.x/2);
% wp(4,:) = 0:1:n-1; % Time
% wp(5:7,:) = zeros([3,n]); % Velocity boundary conditions
wp(1,:) = [-2.0 0 2.0];
wp(2,:) = [0.0 2.0 0];
wp(3,:) = ones([1,3])*(base.l + l1.x/2);
wp(4,:) = 0:1:3-1; % Time
wp(5:7,:) = zeros([3,3]); % Velocity boundary conditions
%% Import robot
Ts = 0.001;
[scara,scara_info] = importrobot("main_basic_scara_ss_bodytree");