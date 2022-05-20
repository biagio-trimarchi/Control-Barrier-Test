function ds = droneDynamics(s, u, params)
    %% Extract variables (for clarity)
    m = params.m;
    g = params.g;
    J = params.J;
    
    x = s(1);
    y = s(2);
    theta = s(3);
    dx = s(4);
    dy = s(5);
    w = s(6);
    
    %% Compute Dynamics
    ds(1) = dx;
    ds(2) = dy;
    ds(3) = w;
    
    ds(4) = u(1)/m * sin(theta);
    ds(5) = u(1)/m * cos(theta) - g;
    ds(6) = -u(2)/J;
end