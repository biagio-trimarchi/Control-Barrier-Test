function u = controllerAPF(x, goal, obstacles, n_obstacles, Katt, Krep, rho0)
    u = -deltaUatt(x, goal, Katt);
    
    for i=1:n_obstacles
        if rho(x, obstacles(i)) <= rho0
            u = u - deltaUrep(x, obstacles(i), Krep, rho0);
        end
    end
end