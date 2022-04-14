function plotDisk(center, radius)

    theta = 0:0.1:2*pi;
    fill(center(1) + radius*cos(theta), center(2) + radius*sin(theta), 'k')

end