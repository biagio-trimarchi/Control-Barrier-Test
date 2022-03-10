function dh = deltah(x, obs)
    dh = [2 * (x-obs.position)', 0, 0];
end