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
n = 4;
d_safe = 1;
x_start = [0; 0; 25];
x_goal = [20; 20; 25];

%% OBSTACLES
obs(1).center = [10; 10; 25];
obs(1).radius = 2;
[obs(1).sphereX, obs(1).sphereY, obs(1).sphereZ] = sphere();

%% SAFETY FILTER PREPARATION
syms s [n*d 1]
syms aux

F = [zeros(d), eye(d); zeros(d), zeros(d)];
G = [zeros(d); eye(d)];

f = F*s;
g = G;

h1(s) = (s(1:d) - obs(1).center)'*(s(1:d) - obs(1).center) - (d_safe + obs(1).radius)^2;
alpha1(aux) = [aux; aux; aux];

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

lim = 5;
A1 = matlabFunction(-lieDer(lieDerOrdr(h1, f, s, n-1), g, s), 'Vars', {[s1 s2 s3 s4 s5 s6]});
A = [1 0 0;
    -1 0 0;
     0 1 0;
     0 -1 0;
     0 0 1;
     0 0 -1];
tmp = alpha1(psi1(n));
b1 = matlabFunction(lieDerOrdr(h1, f, s, n)+bigO1+tmp(n+1), 'Vars', {[s1 s2 s3 s4 s5 s6]});
b = [lim; lim; lim; lim; lim; lim];

K = [15, 10, 100, 100];

dt = 0.01;
T = 5;
for tt = 1:5/dt
    
end
