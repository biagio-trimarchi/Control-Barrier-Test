function r = rho(x, obstacle)
    r = norm(x-obstacle.position) - obstacle.radius;
end