function u = controllerDF(s, params, sigma, dsigma, ddsigma, dddsigma)
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
    zB = R(:, 3);
    
    rr = [x; y; z];
    v = [dx; dy; dz];
    rr_des = [sigma(1); sigma(2); sigma(3)];
    v_des = [dsigma(1); dsigma(2); dsigma(3)];
    a_des = [ddsigma(1); ddsigma(2); ddsigma(3)];
    da_des = [dddsigma(1); dddsigma(2); dddsigma(3)];
    
    yaw_des = sigma(4);
    dyaw_des = dsigma(4);
    
    %% FORCE
    K_p = 10;
    K_v = 5;
    
    F_des = -K_p*(rr- rr_des) - K_v*(v - v_des) + m*[0; 0; g] + m*a_des;
    u(1, 1) = F_des' * zB;
    
    %% MOMENTS
    zB_des = F_des/norm(F_des);
    
    xC_des = [cos(yaw_des); sin(yaw_des); 0];
    
    yB_des = cross(zB_des, xC_des)/norm(cross(zB_des, xC_des));
    xB_des = cross(yB_des, zB_des);
    
    R1_des = [xB_des, yB_des, zB_des];
    R2_des = [-xB_des, -yB_des, zB_des];
    
    eR = 1/2*veeMap(R1_des'*R - R'*R1_des);
    
    hw = m/u(1) * (da_des - (zB_des'*da_des)*zB_des);
    p_des = -hw'*yB_des;
    q_des = hw'*xB_des;
    r_des = dyaw_des*[0, 0, 1]*zB_des;
    
    w_des = [p_des; q_des; r_des];
    ew = w - w_des;
    
    KR = 100;
    Kw = 50;
    M = - KR*eR - Kw*ew;
    
    u(2) = M(1);
    u(3) = M(2);
    u(4) = M(3);
    
    %% UTILITIES
    function w = veeMap(R)
        w(1, 1) = R(3, 2);
        w(2, 1) = R(1, 3);
        w(3, 1) = R(2, 1);
    end
end