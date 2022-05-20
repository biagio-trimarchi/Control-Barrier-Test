clc
clear
addpath('./Functions')

%% DRONE PARAMETERS
params.m = 0.030;  
params.g = 0;   
params.J = [1.43e-5,   0,          0; 
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];

s(:, 1) = zeros(12, 1);

dt = 0.01;
for tt = 1:10/dt
    to = dt*(tt-1);
    tf = dt*tt;
    
    u = [params.m*params.g; 0.00002; 0; 0.0];
    odefun = @(t, s) droneModel(s, params, u);
    [~, y] = ode45(odefun, [to tf], s(:, tt));
    s(:, tt+1) = y(end, :)';
end

i = 1;
for tt = 1:5:length(s(1, :))
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
    
    figure(1)
    plot3(0, 0, 0)
    hold on
    plot3([x-ex(1)/2 x+ex(1)/2], [y-ex(2)/2 y+ex(2)/2], [z-ex(3)/2 z+ex(3)/2])
    plot3([x-ey(1)/2 x+ey(1)/2], [y-ey(2)/2 y+ey(2)/2], [z-ey(3)/2 z+ey(3)/2])
    plot3([x x+ez(1)/4], [y y+ez(2)/4], [z z+ez(3)/4])
    plot3(pos(1, :), pos(2, :), pos(3, :));
    hold off
    drawnow
    
    i=i+1;
end