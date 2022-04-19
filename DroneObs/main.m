%% %% %% DRONE + CBF

%% SETUP
clear
clc
addpath('./Functions')

%% PARAMETERS
gf = 9.81;
m = 1;
J = eye(3);
d = 3;
n = 2;
d_safe = 1;
x_start = [0; 0; 0];
x_goal = [20; 20; 25];

%% OBSTACLES
obs(1).center = [10; 10; 10];
obs(1).radius = 5;
[obs(1).sphereX, obs(1).sphereY, obs(1).sphereZ] = sphere();

%% SAFETY FILTER PREPARATION
syms s [n*d 1]
syms aux

F = [zeros(d), eye(d); zeros(d), zeros(d)];
G = [zeros(d); eye(d)];

f = F*s;
g = G;

h1(s) = (s(1:d) - obs(1).center)'*(s(1:d) - obs(1).center) - (d_safe + obs(1).radius)^2;
alpha1(aux) = [aux^3; aux^3; aux^3];

psi1(1) = h1;
for i = 2:n
    tmp = alpha1(psi1(i-1));
    psi1(i) = lieDer(psi1(i-1), F*s, s) + tmp(i);
end

bigO1 = 0;
for i = 1:n-1
    tmp = alpha1(psi1(n-i));
    bigO1 = bigO1 + lieDerOrdr(tmp(n-i+1),f, s, i);
end

A1 = -lieDer(lieDerOrdr(h1, f, s, n-1), g, s);
tmp = alpha1(psi1(n));
b1 = lieDerOrdr(h1, f, s, n)+bigO1+tmp(n+1);

K = [10, 5, 5];
dt = 0.0001;
t = 0:dt:1;
ss(:, 1) = [x_start; zeros(d, 1)];
xx(:, 1) = [ss(:, 1); zeros(6, 1)];
for tt = 1:length(t)
    u(1, 1) = -K(1)*(ss(1, tt) - x_goal(1)) - K(2)*ss(4, tt);
    u(2, 1) = -K(1)*(ss(2, tt) - x_goal(2)) - K(2)*ss(5, tt);
    u(3, 1) = -K(1)*(ss(3, tt) - x_goal(3)) - K(2)*ss(6, tt);
    
    temp = (num2cell(ss(:, tt)));
    options = optimoptions('quadprog', 'Display', 'off');
    tic
    u = quadprog(eye(d), -2*u, double(A1(temp{:})), double(b1(temp{:})), [], [], [], [], u, options);
    elapsed(tt) = toc;
    [F, M] = ctrlSmallAngles(xx(:, tt), 0, 0, 0, 0, m, gf, u);
    dx = droneDynRPY(m, J, gf, xx(:, tt), F, M)';
    xx(:, tt+1) = xx(:, tt) + dx*dt;
    ss(:, tt+1) = xx(1:6, tt+1);
    %ss(:, tt+1) = ss(:, tt) + (F*ss(:, tt) + G*u)*dt;
end

plot3(ss(1, :), ss(2, :), ss(3, :))
hold on
surf(obs(1).sphereX*obs(1).radius + obs(1).center(1), ...
     obs(1).sphereY*obs(1).radius + obs(1).center(2), ...
     obs(1).sphereZ*obs(1).radius + obs(1).center(3))
surf(obs(1).sphereX*(obs(1).radius + d_safe) + obs(1).center(1), ...
     obs(1).sphereY*(obs(1).radius + d_safe) + obs(1).center(2), ...
     obs(1).sphereZ*(obs(1).radius + d_safe) + obs(1).center(3), ...
     'FaceAlpha', 0.1) 
hold off
