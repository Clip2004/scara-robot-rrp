clear all
clc
close all

%% Parameters
r1 = 89.537/10; % Longitud barra 1 [cm]
r2 = 15.6463;   % Longitud barra 2 [cm]

%% Initial Conditions DK
% Inicializar los rangos de los ángulos
angulo_brazo1 = -90:1:90;  % Ángulo del brazo 1 (de -90 a 90 grados)
angulo_brazo2 = -90:1:90;  % Ángulo del brazo 2 (de -90 a 90 grados)

% Crear una cuadrícula de combinaciones de ángulos
[grid_brazo1, grid_brazo2] = meshgrid(angulo_brazo1, angulo_brazo2);

% Convertir las matrices en vectores columna
TH = [grid_brazo1(:), grid_brazo2(:)];

% Convertir los ángulos de la matriz TH de grados a radianes
TH_radianes = deg2rad(TH);

th1 = TH_radianes(:,1);
th2 = TH_radianes(:,2);

%% Cálculo de la posición del extremo del brazo
ly = r1*cos(th1) + r2*cos(th1 + th2);
lx = r1*sin(th1) + r2*sin(th1 + th2);

%% Gráfica de los puntos
figure;
plot(lx, ly, 'b.', 'MarkerSize', 2);
title('Trayectoria del extremo del brazo');
xlabel('Posición X [cm]');
ylabel('Posición Y [cm]');
ylim([-30, 30])
axis equal;
grid on;
hold on;

%% Agregar semicircunferencia
% Definir dos puntos para la semicircunferencia
x1 = 20; y1 = 4;  % Coordenadas del primer punto
x2 = 20; y2 = 10; % Coordenadas del segundo punto

% Calcular la distancia y el radio
d = sqrt((x2 - x1)^2 + (y2 - y1)^2);
R = d / 2; % Radio como la mitad de la distancia

% Calcular el punto medio entre x1 y x2
Mx = (x1 + x2) / 2;
My = (y1 + y2) / 2;

% Calcular la altura desde el punto medio al centro
h = sqrt(R^2 - (d / 2)^2);

% Vector unitario perpendicular al segmento entre (x1, y1) y (x2, y2)
dx = x2 - x1;
dy = y2 - y1;
u_x = -dy / d;
u_y = dx / d;

% Calcular el centro de la semicircunferencia
Cx = Mx + h * u_x;
Cy = My + h * u_y;

%% Generar la semicircunferencia
theta = linspace(atan2(y1 - Cy, x1 - Cx), atan2(y2 - Cy, x2 - Cx), 60);

% CORRECCIÓN: x e y correctamente asignados
x_semi = Cx + R * cos(theta);  % coordenada X
y_semi = Cy + R * sin(theta);  % coordenada Y

% Graficar la semicircunferencia
plot(x_semi, y_semi, 'r-', 'LineWidth', 1.5);
plot([x1 x2], [y1 y2], 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(Cx, Cy, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% legend('Trayectoria del brazo', 'Semicircunferencia', 'Puntos extremos', 'Centro de la semicircunferencia',Location='southeast');
hold on;
%% Línea recta entre los dos puntos
x1_l = 0; y1_l = 20;
x2_l = 10; y2_l = 20;
plot([x1_l, x2_l], [y1_l, y2_l], 'k-', 'LineWidth', 1.5);
% Calcular puntos intermedios en la línea recta
num_puntos_intermedios = 20; % Número de puntos intermedios
x_intermedios = linspace(x1_l, x2_l, num_puntos_intermedios);
y_intermedios = linspace(y1_l, y2_l, num_puntos_intermedios);
% Trazar los puntos intermedios
plot(x_intermedios,y_intermedios, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
% Marcar los puntos extremos y el centro de la semicircunferencia
plot(x1_l, y1_l, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Punto inicial
plot(x2_l, y2_l , 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % Punto final
hold on;
%%
Ts = 0.1;
[scara_bodytree,ms2r_info] = importrobot('main_scara_ss_bodytree.slx');
% wp(1,:) = [   0, -5.15, -4.94, 0.18,  5.06,  4.96]/100; % trayectoria de x
% wp(2,:) = [24.6, 22.85, 20.25, 20.4, 20.33, 22.69]/100; % y
%% Waypoints corregidos
n = length(x_semi);  % número de puntos (60)

wp(1,:) = x_semi ;           % X en metros
wp(2,:) = y_semi;           % Y en metros
wp(3,:) = 0.0 * ones(1, n);   % Z constante en metros
% n = length(wp)
% CORRECCIÓN: usar n (número de columnas) no length(wp)
wp(4,:) = (0:n-1) / 10;           % Tiempos: 0, 0.1, 0.2, ..., 5.9 seg
% %%
% tiempo = out.angulos.Time;
% datos = rad2deg(out.angulos.Data)+90;
% % plot(datos(:,1),datos(:,2))
% %%
% % Combina los datos de tiempo y datos en una sola matriz
% datosGuardar = [tiempo, datos];
% 
% % Guarda la matriz en un archivo .txt
% writematrix(datosGuardar, 'angulos.txt', 'Delimiter', 'tab'); % Usa tabulación como delimitador
%%
tiempo = wp(4,:)'
datos = (wp(1:2,:)*100)'
% plot(datos(:,1),datos(:,2))
%%
% Combina los datos de tiempo y datos en una sola matriz
datosGuardar = [tiempo, datos];

% Guarda la matriz en un archivo .txt
writematrix(datosGuardar, 'puntos_circulo_60.txt', 'Delimiter', 'tab'); % Usa tabulación como delimitador
