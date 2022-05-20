clear
clc

addpath('./Functions')

dt = 0.0001;
t = 0:dt:4*pi;
xx(:, 1) = zeros(12, 1);
g = 9.81;
m = 1;
J = eye(3);

%% CIRCLE
f = 2;
circle = [10*cos((t/f)); 10*sin((t/f)); 5*ones(1, length(t))];
vel_circle = [-10/f*sin((t/f)); 10/f*cos((t/f)); zeros(1, length(t))];
acc_circle = [-10/(f^2)*cos((t/f)); -10/(f^2)*sin((t/f)); zeros(1, length(t))];

for tt = 1:length(t)
    psi_des(tt) = vel_circle(1, tt)/norm(vel_circle(:, tt));
end
psi_des(1) = 0;

%% HOVER


Kp = 10;
Kv = 5;

xx(1:3, 1) = circle(:, 1);
xx(9, 1) = psi_des(1);
for tt = 1:length(t)
    u = Kp*(circle(:, tt) - xx(1:3, tt)) + Kv*(vel_circle(:, tt) - xx(4:6, tt)) + acc_circle(:, tt);
    %u = [0; 0; 0];
    [F, M] = ctrlSmallAngles(xx(:, tt), psi_des(tt), 0, 0, 0, m, g, u);
    el(tt, :) = M;
    dx = droneDynRPY(m, J, g, xx(:, tt), F, M)';
    xx(:, tt+1) = xx(:, tt) + dx*dt;
end

for tt = 1:200:length(t)
    x = xx(1, tt);
    y = xx(2, tt);
    z = xx(3, tt);
    dx = xx(4, tt);
    dy = xx(5, tt);
    dz = xx(6, tt);
    
    phi = xx(7, tt);
    theta = xx(8, tt);
    psi = xx(9, tt);
     
    p = xx(10, tt);
    q = xx(11, tt);
    r = xx(12, tt);
    
    R = wRb(phi, theta, psi);
    ex = R*[1; 0; 0];
    ey = R*[0; 1; 0];
    ez = R*[0; 0; 1];
    
    figure(1)
    plot3(0, 0, 0)
    hold on
    plot3([x-ex(1)/2 x+ex(1)/2], [y-ex(2)/2 y+ex(2)/2], [z-ex(3)/2 z+ex(3)/2])
    plot3([x-ey(1)/2 x+ey(1)/2], [y-ey(2)/2 y+ey(2)/2], [z-ey(3)/2 z+ey(3)/2])
    plot3([x x+ez(1)/4], [y y+ez(2)/4], [z z+ez(3)/4])
    
    plot3(circle(1, :), circle(2, :), circle(3, :))
    hold off
    drawnow
end
