%% %% %% COMPARISON BETWEEN APF AND BF %%%

%% SETUP
clear
clc
addpath('Functions')

%% PARAMETERS
x_0 = [0; 0; 0; 0];
x_goal = [3; 5];

n_obstacles = 2;
K_1 = 1;
K_2 = 1;
freq = 1000;
dt = 1/freq;
t = 0:dt:10;

%% OBSTACLES
obstacles(1).position = [1; 2];
obstacles(1).radius = 0.5;

obstacles(2).position = [2.5; 3];
obstacles(2).radius = 0.5;

%% LINEAR SYSTEM
A = [zeros(2), eye(2); zeros(2), zeros(2)];
B = [zeros(2); eye(2)];

%% CONTROL
x(:, 1) = x_0;
for tt = 1:length(t)
    u_nom = -K_2*(x(3:4, tt)) - K_1*(x(1:2, tt) - x_goal);
    
    
    h1 = h(x(1:2, tt), obstacles(1));
    dh1 = deltah(x(1:2, tt), obstacles(1));
    ddh1 = nablah();
    xx = x(:, tt);
    bobs1 = xx'*A'*ddh1*A*xx + dh1*A^2*xx + 2*h1*dh1*A*xx + (dh1*A*xx)^2 + 2*h1^2*dh1*A*xx + h1^4;
    Aobs1 = dh1*A*B;
    h1 = h(x(1:2, tt), obstacles(2));
    dh1 = deltah(x(1:2, tt), obstacles(2));
    ddh1 = nablah();
    xx = x(:, tt);
    bobs2 = xx'*A'*ddh1*A*xx + dh1*A^2*xx + 2*h1*dh1*A*xx + (dh1*A*xx)^2 + 2*h1^2*dh1*A*xx + h1^4;
    Aobs2 = dh1*A*B;
    
    options = optimoptions('quadprog', 'Display', 'off');
    u = quadprog(eye(2), -u_nom', -[Aobs1;Aobs2], [bobs1; bobs2], [], [], [], [], [], options);
    
    x(1:2, tt+1) = x(1:2, tt) + x(3:4, tt)*dt;
    x(3:4, tt+1)= x(3:4, tt) + u*dt;
end

plot(x(1, :), x(2, :));
hold on
plot(obstacles(1).position(1), obstacles(1).position(2), '.', 'MarkerSize', 200)
plot(obstacles(2).position(1), obstacles(2).position(2),  '.', 'MarkerSize', 200)
hold off