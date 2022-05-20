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

F = [zeros((n-1)*d), eye(d); zeros(1, n)];

G = [zeros(n-1, 1); 1];

f = F*s;
g = G;

h1(s) = (s(1:d) - obs(1).center)'*(s(1:d) - obs(1).center) - (d_safe + obs(1).radius)^2;
alpha1(aux) = [aux; aux; aux; aux; aux];

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
A1 = matlabFunction(-lieDer(lieDerOrdr(h1, f, s, n-1), g, s), 'Vars', {[s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12]});
A = [1 0 0;
    -1 0 0;
     0 1 0;
     0 -1 0;
     0 0 1;
     0 0 -1];
tmp = alpha1(psi1(n));
b1 = matlabFunction(lieDerOrdr(h1, f, s, n)+bigO1+tmp(n+1), 'Vars', {[s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12]});
b = [lim; lim; lim; lim; lim; lim];

K = [15, 10, 100, 100];
dt = 0.0001;
t = 0:dt:5;
ss(:, 1) = [x_start; zeros((n-1)*d, 1)];
xx(:, 1) = [ss(:, 1); zeros(6, 1)];
for tt = 1:length(t)
    u(1, 1) = -K(1)*(ss(1, tt) - x_goal(1)) - K(2)*ss(4, tt) - K(3)*K(2)*ss(7, tt) - K(4)*ss(10, tt);
    u(2, 1) = -K(1)*(ss(2, tt) - x_goal(2)) - K(2)*ss(5, tt) - K(3)*K(2)*ss(8, tt) - K(4)*ss(11, tt);
    u(3, 1) = -K(1)*(ss(3, tt) - x_goal(3)) - K(2)*ss(6, tt) - K(3)*K(2)*ss(9, tt) - K(4)*ss(12, tt);
    psi_des = (x_goal - x_start)/norm(x_goal - x_start);
    psi_des = 0;
    
    u(1, 1) = min(lim, max(-lim, u(1, 1)));
    u(2, 1) = min(lim, max(-lim, u(2, 1)));
    u(3, 1) = min(lim, max(-lim, u(3, 1)));
    
    
    
    options = optimoptions('quadprog', 'Display', 'off');
    tic
    u = quadprog(eye(d), -2*u, A1(ss(:, tt)'), b1(ss(:, tt)'), [], [], [], [], u, options);
%     elapsed(tt) = toc;
%     el(tt, :) = u;
%     [F, M] = ctrlSmallAngles(xx(:, tt), 0, 0, 0, 0, m, gf, u);
%     dx = droneDynRPY(m, J, gf, xx(:, tt), F, M)';
%     xx(:, tt+1) = xx(:, tt) + dx*dt;
%    ss(:, tt+1) = xx(1:6, tt+1);
    ss(:, tt+1) = ss(:, tt) + (F*ss(:, tt) + G*u)*dt;
end

%% PLOTS
figure(1)
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

% for tt = 1:500:length(t)
%     x = xx(1, tt);
%     y = xx(2, tt);
%     z = xx(3, tt);
%     dx = xx(4, tt);
%     dy = xx(5, tt);
%     dz = xx(6, tt);
%     
%     phi = xx(7, tt);
%     theta = xx(8, tt);
%     psi = xx(9, tt);
%      
%     p = xx(10, tt);
%     q = xx(11, tt);
%     r = xx(12, tt);
%     
%     R = wRb(phi, theta, psi);
%     ex = R*[1; 0; 0];
%     ey = R*[0; 1; 0];
%     ez = R*[0; 0; 1];
%     
%     figure(2)
%     plot3(0, 0, 0)
%     hold on
%     plot3(ss(1, :), ss(2, :), ss(3, :))
%     surf(obs(1).sphereX*obs(1).radius + obs(1).center(1), ...
%          obs(1).sphereY*obs(1).radius + obs(1).center(2), ...
%         obs(1).sphereZ*obs(1).radius + obs(1).center(3))
%     surf(obs(1).sphereX*(obs(1).radius + d_safe) + obs(1).center(1), ...
%          obs(1).sphereY*(obs(1).radius + d_safe) + obs(1).center(2), ...
%         obs(1).sphereZ*(obs(1).radius + d_safe) + obs(1).center(3), ...
%         'FaceAlpha', 0.1) 
%     plot3([x-ex(1)/2 x+ex(1)/2], [y-ex(2)/2 y+ex(2)/2], [z-ex(3)/2 z+ex(3)/2])
%     plot3([x-ey(1)/2 x+ey(1)/2], [y-ey(2)/2 y+ey(2)/2], [z-ey(3)/2 z+ey(3)/2])
%     plot3([x x+ez(1)/4], [y y+ez(2)/4], [z z+ez(3)/4])
%     
%     hold off
%     drawnow
% end
