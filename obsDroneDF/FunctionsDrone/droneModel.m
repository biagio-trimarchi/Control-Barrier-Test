function ds = droneModel(s, params, u)
    %% AUXILIARY VARIABLES
    m = params.m;
    J = params.J;
    g = params.g;
    
    x = s(1);
    y = s(2);
    z = s(3);
    dx = s(4);
    dy = s(5);
    dz = s(6);
    phi = s(7);
    theta = s(8);
    psi = s(9);
    p = s(10);
    q = s(11);
    r = s(12);
    
    w = [p; q; r];
    
    Rpqr = [cos(theta) 0 -cos(phi)*sin(theta);
        0 1 sin(phi);
        sin(theta) 0 cos(phi)*cos(theta)];
    
    R = wRb(phi, theta, psi);
    %% TRANSLATION
    % Velocity
    ds(1, 1) = dx;
    ds(2, 1) = dy;
    ds(3, 1) = dz;
    
    % Acceleration
    aux = [0; 0; -g] + (1/m)*R*[0; 0; u(1)];
    
    ds(4, 1) = aux(1);
    ds(5, 1) = aux(2);
    ds(6, 1) = aux(3);
    
    %% ROTATION
    % Velocity
    aux = Rpqr\w;
    
    ds(7, 1) = aux(1);
    ds(8, 1) = aux(2);
    ds(9, 1) = aux(3);
    
    % Acceleration
    aux = J\([u(2); u(3); u(4)] - cross(w,J*w));
    
    ds(10, 1) = aux(1);
    ds(11, 1) = aux(2);
    ds(12, 1) = aux(3);
end