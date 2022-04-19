function [F, M] = ctrlSmallAngles(s, psi_des, p_des, q_des, r_des, m, g, u)
    %% STATE
    phi = s(7);
    theta = s(8);
    psi = s(9);
     
    p = s(10);
    q = s(11);
    r = s(12);
    
    %% POSITION CONTROLLER
    F = m*(u(3) + g);
    
    %% ATTITUDE CONTROLLER
    % Parameters
    K_p_phi = 10;
    K_d_phi = 5;
    K_p_theta = 10;
    K_d_theta = 5;
    K_p_psi = 10;
    K_d_psi = 5;
    
    % Desired Attitude
    phi_des = 1/g * (u(1)*sin(psi) - u(2)*cos(psi));
    theta_des = 1/g * ((1)*cos(psi) + u(2)*sin(psi));
    
    % Control Input
    M(1) = K_p_phi*(phi_des - phi) + K_d_phi*(p_des - p);
    M(2) = K_p_theta*(theta_des - theta) + K_d_theta*(q_des - q);
    M(3) = K_p_psi*(psi_des - psi) + K_d_psi*(r_des - r);
end