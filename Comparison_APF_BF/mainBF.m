%% %% %% COMPARISON BETWEEN APF AND BF %%%

%% SETUP
clear
clc
addpath('Functions')

%% PARAMETERS
x_0 = [0; 0];
x_goal = [3; 5];

d = 2;
n_obstacles = 2;
alpha = 0.5;
K_att = 1;
freq = 100;
dt = 1/freq;
t = 0:dt:10;

%% OBSTACLES
obstacles(1).position = [1; 2];
obstacles(1).radius = 0.5;

obstacles(2).position = [2.5; 3];
obstacles(2).radius = 0.5;

%% CONTROL
x(:, 1) = x_0;
for tt = 1:length(t)
    v_des = -K_att*(x(:, tt) - x_goal);
    H = eye(2);
    F = -v_des';
    A = -[deltah(x(:,tt), obstacles(1))'; deltah(x(:, tt), obstacles(2))'];
    b = alpha*[h(x(:, tt), obstacles(1))^2; h(x(:, tt), obstacles(2))];
    
    options = optimoptions('quadprog', 'Display', 'off');
    u = quadprog(H, F, A, b, [], [], [], [], [], options);
    
    x(:, tt+1)= x(:, tt) + u*dt;
end

plot(x(1, :), x(2, :), 'LineWidth', 2);
hold on
plot(x_0(1), x_0(2), '.', 'MarkerSize', 50)
plot(x_goal(1), x_goal(2), '.', 'MarkerSize', 50)

plot(obstacles(1).position(1), obstacles(1).position(2), '.', 'Color', 'k', 'MarkerSize', 200)
plot(obstacles(2).position(1), obstacles(2).position(2),  '.', 'Color', 'k', 'MarkerSize', 200)
hold off

legend('Trajectory', 'Start Position', 'Target Position', 'Obstacle', 'Location', 'northwest')

xlim([-1 4])
ylim([-1 6])