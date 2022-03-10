function h = h(x, obs)
    h = (x-obs.position)'*(x-obs.position) - obs.radius^2;
end