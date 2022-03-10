function u = deltaUrep(x, obstacle, K, rho0)
    u = K/(rho(x, obstacle)^3) * (1/rho(x, obstacle) - 1/rho0)*-(x - obstacle.position);
end