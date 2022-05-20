clc
clear
addpath('./Functions')

%% DRONE PARAMETERS
params.m = 0.030;  
params.g = 9.81;   
params.J = [1.43e-5,   0,          0; 
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];

%% GOING UP 
scale = 10;
sigma = @(t) [0; 0; t^4/10; 0];
dsigma = @(t) [0; 0;4*t^3/10; 0];
ddsigma = @(t) [0; 0; 12*t^2/10; 0];
dddsigma = @(t) [ 0; 0; 24*t/10; 0];

%% HOVER AND ROTATE
scale = 2;
sigma = @(t) [0; 0; 0;  t/scale];
dsigma = @(t) [0; 0; 0; 1/scale];
ddsigma = @(t) [0; 0; 0; 0];
dddsigma = @(t) [0; 0; 0; 0];

%% CIRCLE
scale = 10;
sigma = @(t) [cos(2*pi/scale*t); sin(2*pi/scale*t); 0;  0];
dsigma = @(t) [-2*pi*sin(2*pi/scale*t)/scale; 2*pi*cos(2*pi/scale*t)/scale; 0; 0];
ddsigma = @(t) [-4*(pi^2)*cos(2*pi/scale*t)/(scale^2); -4*(pi^2)*sin(2*pi/scale*t)/(scale^2); 0; 0];
dddsigma = @(t) [8*(pi^3)*sin(2*pi/scale*t)/(scale^3); -8*(pi^3)*cos(2*pi/scale*t)/(scale^3); 0; 0];

aux = sigma(0);
s(1:3, 1) = aux(1:3);

aux = dsigma(0);
s(4:6, 1) = aux(1:3);

s(7:12) = zeros(6, 1);

dt = 0.01;
for tt = 1:10/dt
    tt
    to = dt*(tt-1);
    tf = dt*tt;
    
    odefun = @(t, s) simulation(t, s, params, sigma, dsigma, ddsigma, dddsigma);
    [ta, y] = ode45(odefun, [to tf], s(:, tt));
    s(:, tt+1) = y(end, :)';
end

i = 1;
for tt = 1:length(s(1, :))
    x = s(1, tt);
    y = s(2, tt);
    z = s(3, tt);
    dx = s(4, tt);
    dy = s(5, tt);
    dz = s(6, tt);
    
    phi = s(7, tt);
    theta = s(8, tt);
    psi = s(9, tt);
     
    p = s(10, tt);
    q = s(11, tt);
    r = s(12, tt);
    
    R = wRb(phi, theta, psi);
    ex = R*[1; 0; 0];
    ey = R*[0; 1; 0];
    ez = R*[0; 0; 1];
    
    pos(:, i) = [x; y; z];
    aux = sigma(i*dt);
    pos1(:, i) = aux(1:3);
    
    figure(1)
    plot3(0, 0, 0)
    hold on
    plot3([x-ex(1)/2 x+ex(1)/2], [y-ex(2)/2 y+ex(2)/2], [z-ex(3)/2 z+ex(3)/2])
    plot3([x-ey(1)/2 x+ey(1)/2], [y-ey(2)/2 y+ey(2)/2], [z-ey(3)/2 z+ey(3)/2])
    plot3([x x+ez(1)/4], [y y+ez(2)/4], [z z+ez(3)/4])
    plot3(pos(1, :), pos(2, :), pos(3, :));
    plot3(pos1(1, :), pos1(2, :), pos1(3, :));
    hold off
    drawnow
    
    i=i+1;
end