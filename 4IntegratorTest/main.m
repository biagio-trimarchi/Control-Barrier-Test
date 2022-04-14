%% %% %% 4^th INTEGRATOR

%% SETUP
clear
clc
addpath('./Functions')

%% PARAMETERS
d = 2;  % Dimension
n = 3;  % Integrator Order
x_goal = [5; 5];
obs_c1 = [2; 2.5];
obs_r1 = 0.5;
obs_c2 = [5; 3];
obs_r2 = 1;
d_safe = 0.5;

%% DYNAMICS
F = [zeros(n-1, 1), eye(n-1); zeros(1, n)];
F = kron(F, eye(d));

G = [zeros(n-1, 1); 1];
G = kron(G, eye(d));

%plotDisk(obs_c, obs_r);

syms x [d*n 1] 
syms aux
f = F*x;
g = G;

h1(x) = (x(1:2) - obs_c1)'*(x(1:2) - obs_c1) - (d_safe+obs_r1)^2;

alpha1(aux) = [aux;
             3*aux;
             6*aux;
             10*aux];


psi1(1) = h1;
for i = 2:n
    tmp = alpha1(psi1(i-1));
    psi1(i) = lieDer(psi1(i-1), F*x, x) + tmp(i);
end

bigO1 = 0;
for i = 1:n-1
    tmp = alpha1(psi1(n-i));
    bigO1 = bigO1 + lieDerOrdr(tmp(n-i+1),f, x, i);
end

A1 = -lieDer(lieDerOrdr(h1, f, x, n-1), g, x);
tmp = alpha1(psi1(n));
b1 = lieDerOrdr(h1, f, x, n)+bigO1+tmp(n+1);

h2(x) = (x(1:2) - obs_c2)'*(x(1:2) - obs_c2) - (d_safe+obs_r2)^2;

alpha2(aux) = [aux;
             3*aux;
             6*aux;
             10*aux];

psi2(1) = h2;
for i = 2:n
    tmp = alpha2(psi2(i-1));
    psi2(i) = lieDer(psi2(i-1), F*x, x) + tmp(i);
end

bigO2 = 0;
for i = 1:n-1
    tmp = alpha2(psi2(n-i));
    bigO2 = bigO2 + lieDerOrdr(tmp(n-i+1),f, x, i);
end

A2 = -lieDer(lieDerOrdr(h2, f, x, n-1), g, x);
tmp = alpha2(psi2(n));
b2 = lieDerOrdr(h2, f, x, n)+bigO2+tmp(n+1);

K = [50, 40, 50, 80];
dt = 0.01;
t = 0:dt:5;
xx(:, 1) = zeros(n*d, 1);
for tt = 1:length(t)
    u(1, 1) = -K(1)*(xx(1, tt) - x_goal(1)) - K(2)*xx(3, tt) - K(3)*xx(5, tt);
    u(2, 1) = -K(1)*(xx(2, tt) - x_goal(2)) - K(2)*xx(4, tt) - K(3)*xx(6, tt);
    
    temp = (num2cell(xx(:, tt)));
    options = optimoptions('quadprog', 'Display', 'off');
    tic
    u = quadprog(eye(d), -2*u, [double(A1(temp{:})); double(A2(temp{:}))], [double(b1(temp{:}));double(b2(temp{:}))], [], [], [], [], u, options);
    elapsed(tt) = toc;
    xx(:, tt+1) = xx(:, tt) + (F*xx(:, tt) + G*u)*dt;
end

figure(1)
plot(xx(1,:), xx(2, :), 'LineWidth', 2);
hold on
plotDisk(obs_c1, obs_r1);
plotDisk(obs_c2, obs_r2);
hold off