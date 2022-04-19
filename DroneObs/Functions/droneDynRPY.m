function ds = droneDynRPY(m, J, g, s, F, M)
    % [x y z dx dy dz phi theta psi p q r]
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
    
    %% Translational Dynamics
    % Velocity
    ds(1) = dx;
    ds(2) = dy;
    ds(3) = dz;
    
    % Acceleration
    WRB = wRb(phi, theta, psi);
    aux = [0; 0; -g] + 1/m*WRB*[0; 0; F];
    
    ds(4) = aux(1);
    ds(5) = aux(2);
    ds(6) = aux(3);
    
    %% Rotational Dynamics
    % Angular Velocity
    R = [cos(theta) 0 -cos(phi)*sin(theta);
         0 1 sin(phi);
         sin(theta) 0 cos(phi)*cos(theta)];
    aux = R\w;
    
    ds(7) = aux(1);
    ds(8) = aux(2);
    ds(9) = aux(3);
    
    % Angular Acceleration
    aux = J\(M - cross(w, J*w));
    
    ds(10) = aux(1);
    ds(11) = aux(2);
    ds(12) = aux(3);
end