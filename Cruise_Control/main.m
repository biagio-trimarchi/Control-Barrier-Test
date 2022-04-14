%% %% %% CRUISE CONTROL

%% SETUP
clear 
clc 
addpath('./Functions')

%% PARAMETERS
tau_d = 1.8;
kappa = 5;
vd = 22;
D0 = 80;
vl0 = 16;
vf0 = 18;
gamma_max = 0.3532;
delta_theta_inf = 0.1;
g = 9.81;
m = 1650;
f0 = 0.1;
f1 = 5;
f2 = 0.25;
rho_sc = exp(-5);

T = 60;
dt = 0.1;
t = 0:dt:T;


%% PERTURBATION
delta_theta = 0.1*cos(2*pi*t/20);

%% STATE
vf = zeros(1, length(t));
dvf = zeros(1, length(t));
vl = zeros(1, length(t));
dvl = zeros(1, length(t));
D = zeros(1, length(t));
dD = zeros(1, length(t));

vf(1) = vf0;
vl(1) = vl0;
D(1) = D0;

%% Preallocate input
u = zeros(1, length(t));
u_d = zeros(2, length(t));

%% Acceleration profile
al = zeros(1, length(t));

%% MATRICES
H = [1/m^2 0
    0 rho_sc];

for tt = 1:length(t)-1
    % Compute control input
    % Build Matrices
    Aclf = [2*(vf(tt) - vd)/m, -1];
    bclf = 2*(vf(tt) - vd)*Fr(vf(tt), f0, f1, f2)/m - (vf(tt) - vd)^2;
    
    Azbcf = [tau_d/m, 0];
    bzcvf = tau_d*Fr(vf(tt), f0, f1, f2)/m + (vl(tt) - vf(tt)) + kappa*h(D(tt), tau_d, vf(tt));
    
    F = [Fr(vf(tt), f0, f1, f2)/m^2; 0];
    
    options = optimoptions('quadprog', 'Display', 'off');
    %u_d(:, tt) = quadprog(2*H(1), -2*F(1), Aclf(1), bclf, [], [], [], [], [], options);
    u_d(:, tt) = quadprog(2*H, -2*F, [Aclf; Azbcf], [bclf; bzcvf], [], [], [], [], [], options);
    u(tt) = u_d(1, tt);
    
    % Update Dynamic
    dvl(tt) = al(tt);
    dvf(tt) = -Fr(vf(tt), f0, f1, f2)/m + g*delta_theta(tt) + u(tt)/m;
    dD(tt) = vl(tt) - vf(tt);
    
    vl(tt+1) = vl(tt) + dvl(tt)*dt;
    vf(tt+1) = vf(tt) + dvf(tt)*dt;
    D(tt+1) = D(tt) + dD(tt)*dt;
end

figure(1)
plot(t, vl)
legend('v_l(t)')

figure(2)
plot(t, vf)
hold on
plot(t, vd*ones(1, length(t)))
legend('v_f(t)', 'v_d')
hold off

figure(3)
plot(t, D)
legend('D(t)')

figure(4)
plot(t, h(D, tau_d, vf))
legend('h(x)')

figure(5)
plot(t, vl, 'LineWidth', 2)
hold on
plot(t, vf, 'LineWidth', 2)
plot(t, vd*ones(1, length(t)), 'LineWidth', 2)
hold off
legend('v_{leading}(t)', 'v_{follwer}(t)', 'v_{desired}')
ylim([15, 23])

pos = 0;
% for tt = 1:length(t)-1
%     plot(pos, 0, 'o', 'MarkerSize', 20)
%     hold on
%     plot(pos-D(tt), 0, 'o', 'MarkerSize', 20)
%     quiver(pos-D(tt), 0, vd, 0)
%     quiver(pos-D(tt), 0, vf(tt), 0)
%     hold off
%     xlim([pos-D0, pos+10])
%     ylim([-5, 5])
%     legend('Leading Vehicle', 'Controlled Vehicle', 'v_d', 'v_f(t)')
%     xlabel('Position')
%     drawnow
%     
%     pos = pos + vl(tt)*dt;
% end