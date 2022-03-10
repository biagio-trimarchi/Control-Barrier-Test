%% %% %% COMPARISON BETWEEN APF AND BF %%%

%% SETUP ENVIROMENT
clear
clc
addpath('./Functions')

%% PARAMETERS
d = 2;
n_obstacles = 2;
rho0 = 0.3;
Katt = 1;
Krep = 1;
freq = 1000;
dt = 1/freq;
t = 0:dt:40;

%% OBSTACLES
obstacles(1).position = [1; 2];
obstacles(1).radius = 0.5;

obstacles(2).position = [2.5; 3];
obstacles(2).radius = 0.5;

%% MISSION PARAMTERS
initial_position = [0; 0];
final_position = [3; 5];

u = controllerAPF(initial_position, final_position , obstacles, n_obstacles, Katt, Krep, rho0)*dt;

x = zeros(d, length(t));
x(:, 1) = initial_position;
for i = 1:length(t)-1
    x(:, i+1) = x(:, i) + controllerAPF(x(:, i), final_position , obstacles, n_obstacles, Katt, Krep, rho0)*dt;
end

plot(x(1, :), x(2, :));
hold on
plot(obstacles(1).position(1), obstacles(1).position(2), '.', 'MarkerSize', 200)
plot(obstacles(2).position(1), obstacles(2).position(2),  '.', 'MarkerSize', 200)
hold off