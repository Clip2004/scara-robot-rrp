% 
% % Robot Tree creator - Robotics Toolbox 
% clear all
% clc
% close all
% 
% Ts = 0.001; % hay que poner el tiempo de muestreo para el tiempo de simulación
% [scara_bodytree,ms2r_info] = importrobot('main_scara_ss_bodytree');
% 
% % Puntos guías:
% wp(1,:) = [0.36, 0.30, 0.30, 0.25, 0.25, 0.30, 0.36]; % trayectoria de x
% wp(2,:) = [0.00, 0.02, 0.14, 0.14, 0.02, 0.02, 0.00]; % y
% wp(3,:) = [0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11]; % z
% 
% wp(4,:) = [0,1,2,3,4,5,6]; % Vector de tiempos, en este caso son segundos
% 
% wp(5,:) = [0.00, 0.50, 0.50, 0.50, 0.50, 0.50, 0.00]; % Velocidad en X
% wp(6,:) = [0.00, 0.50, 0.50, 0.50, 0.50, 0.50, 0.00]; % Velocidad en y
% wp(7,:) = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]; % Velocidad en z
% n = size(wp,2);

% UNIVERSIDAD EIA
% Ing. Mecatrónica - Robótica Industrial
% Docente: David Rozo Osorio, I.M., M.Sc.
% Robot Tree creator - Robotics Toolbox 
clear all
clc
close all

Ts = 0.001;
[scara_bodytree,ms2r_info] = importrobot('main_scara_ss_bodytree.slx');
% wp(1,:) = [   0, -5.15, -4.94, 0.18,  5.06,  4.96]/100; % trayectoria de x
% wp(2,:) = [24.6, 22.85, 20.25, 20.4, 20.33, 22.69]/100; % y
wp(1,:) = [5.0000   5.1508  5.5849  6.2500  7.0659  7.9341  8.7500  9.4151  9.8492 10.0000]/100;
wp(2,:) = [20.0000 20.8551 21.6070 22.1651 22.4620 22.4620 22.1651 21.6070 20.8551 20.0000]/100;
wp(3,:) = [ 0.05425 0.05425 0.05425 0.05425 0.05425 0.05425 0.05425 0.05425 0.05425 0.05425];
wp(4,:) = linspace(0,1,length(wp)); % Vector de tiempos, en este caso son segundos
n = length(wp)
