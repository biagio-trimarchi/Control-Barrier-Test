import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gp import GaussianProcess as GP
from scipy.integrate import ode
from cvxopt import solvers, matrix

def findNextGoal(fitter, goal):
    dom_x = np.arange(0, 4, 0.5)
    dom_y = np.arange(0, 5, 0.5)
    for idx_x in range(dom_x.shape[0]):
        for idx_y in range(dom_y.shape[0]):
            if fitter.posterior_mean(np.array([dom_x[idx_x], dom_y[idx_y]])) > 0:
                if fitter.posterior_variance(np.array([dom_x[idx_x], dom_y[idx_y]])) > 0.1:
                    if np.linalg.norm(np.array([dom_x[idx_x], dom_y[idx_y]]) - goal) > 0.01:
                        return np.array([dom_x[idx_x], dom_y[idx_y]])

    return np.array([-1, -1])

def dist(x, obstacle):
    return np.linalg.norm(x-obstacle['Position'])**2 - obstacle['Radius']**2

def h(x, obstacle):
    return np.linalg.norm(x-obstacle['Position'])**2 - obstacle['Radius']**2 - 0.1**2

def deltah(x, obstacle):
    return 2*(x - obstacle['Position'])

def dynamic(x, u):
    return u

def simulation(t, x, xgoal, obstacles, fitter):
    alpha = 0.5
    K = 2
    v = -K*(x- xgoal)

    # QUADRATIC PROGRAMMING
    A = matrix( -np.block([
                [deltah(x, obstacles[0])],
                [deltah(x, obstacles[1])]
    ]) )

    A = matrix( -fitter.posterior_dxmean(x).T )

    b = matrix( alpha*np.block([ [h(x, obstacles[0])],
                         [h(x, obstacles[1])] 
                    ]) )
    
    b = matrix ( fitter.posterior_mean(x) )
    
    H = matrix( np.eye(2) )
    F = matrix( -np.reshape(v, (2, 1)) )

    sol = solvers.qp(H, F, A, b)
    
    u = np.squeeze(np.array(sol['x']))
    return dynamic(x, u)

def main():
    ##############
    ### PARAMETERS
    ##############

    x0 = np.array([0, 0])
    xgoal = np.array([3, 4])

    d = 2                   # dimension
    n_obstacles = 2         # number of obstacles
    alpha = 1.0             # K-function
    dt = 0.01               # time-step
    dSafe = 0               # Safety radius
    sensingRange = 1.5		# Sensing Range


    # OBSTACLES
    obstacles = []
    obstacles.append({'Position': np.array([1, 2]), 'Radius': 0.5})
    obstacles.append({'Position': np.array([2.5, 3]), 'Radius': 0.5})
    obstacles.append({'Position': np.array([3.0, 1.5]), 'Radius': 0.5})
    obstacles.append({'Position': np.array([1.0, 3.5]), 'Radius': 0.5})

    # QUADRATRIC PROGRAMMING
    solvers.options['show_progress'] = False

    # CONTROL
    x = x0.reshape((d, 1))
    last = x0
    previous = x0

    # print(x)
    # x = np.append(x, x0.reshape((d, 1)), 1)
    # print(x)

    # Initializer - Declare Size of x (1 Here) and of y (1 Here)
    fitter = GP(2, 1)

    #fitter.add_sample(x0 - 0.2, min([ h(x0 - 0.2, obstacles[0]), h(x0 - 0.2, obstacles[1])]))
    #fitter.add_sample(x0, min([ h(x0, obstacles[0]), h(x0, obstacles[1])]))
    if min([ dist(x0, obstacles[0]), dist(x0, obstacles[1]), dist(x0, obstacles[2]), dist(x0, obstacles[3])]) < sensingRange:
        # fitter.add_sample(x0 - 0.2, min([ h(x0 - 0.2, obstacles[0]), h(x0 - 0.2, obstacles[1])]))
        fitter.add_sample(x0, min([ h(x0, obstacles[0]), h(x0, obstacles[1]), h(x0, obstacles[2]), h(x0, obstacles[3])]))
    else:
        # fitter.add_sample(x0 - 0.2, np.array(sensingRange))
        fitter.add_sample(x0, np.array(sensingRange))
    #fitter.optimize_hyperparameters()
    fitter.train()

    xgoal = findNextGoal(fitter, x0)
    f = lambda t,x : simulation(t, x, xgoal, obstacles, fitter)
    r = ode(f).set_integrator('dopri5')
    r.set_initial_value(x0, 0)

    counter = 1
    while r.successful() and r.t < 40:
    #while r.successful() and np.linalg.norm(x[:, -1] - xgoal) > 0.1:
        r.integrate(r.t+dt)
        x = np.append(x, (r.y).reshape((d,1)), 1)
        if np.linalg.norm(last - r.y) > 0.1:
            counter = counter + 1
            last = r.y
            if min([ dist(r.y, obstacles[0]), dist(r.y, obstacles[1]), dist(r.y, obstacles[2]), dist(r.y, obstacles[3])]) < sensingRange:
                fitter.add_sample(r.y, min([ h(r.y, obstacles[0]), h(r.y, obstacles[1]), h(r.y, obstacles[2]), h(r.y, obstacles[3])]))
            else:
                fitter.add_sample(r.y, np.array(sensingRange))
            
            # if counter == 10:
            #     fitter.optimize_hyperparameters()
            #     counter = 0
            fitter.train()

        if np.linalg.norm(r.y - xgoal) < 0.1 or np.linalg.norm(r.y - previous) < 0.001:
            xgoal = findNextGoal(fitter, xgoal)
            print(xgoal)
            print(r.t)
            if np.linalg.norm(xgoal - np.array([-1, -1])) < 0.1:
                break
            T = r.t
            x0 = r.y
            f = lambda t,x : simulation(t, x, xgoal, obstacles, fitter)
            r = ode(f).set_integrator('dopri5')
            r.set_initial_value(x0, T)
        previous = r.y


    ### PLOTS
    circle1 = plt.Circle(obstacles[0]['Position'], obstacles[0]['Radius'], color='r')
    circle2 = plt.Circle(obstacles[1]['Position'], obstacles[1]['Radius'], color='r')
    circle3 = plt.Circle(obstacles[2]['Position'], obstacles[2]['Radius'], color='r')
    circle4 = plt.Circle(obstacles[3]['Position'], obstacles[3]['Radius'], color='r')

    fig, ax = plt.subplots()

    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.add_patch(circle3)
    ax.add_patch(circle4)

    plt.plot(x[0,:], x[1,:])

    ax.set_xlim((-1, 4))
    ax.set_ylim((-1, 6))
    plt.gca().set_aspect('equal', adjustable='box')

    plt.show()

    dom_x = np.arange(-1, 5, 0.2)
    dom_y = np.arange(-1, 5, 0.2)
    xx, yy = np.meshgrid(dom_x, dom_y)
    zz = np.empty(xx.shape)
    zv = np.empty(xx.shape)
    zh = np.empty(xx.shape)
    for idx_x in range(dom_x.shape[0]):
        for idx_y in range(dom_y.shape[0]):
            zz[idx_x, idx_y] = fitter.posterior_mean(np.array([dom_x[idx_x], dom_y[idx_y]]))
            zv[idx_x, idx_y] = fitter.posterior_variance(np.array([dom_x[idx_x], dom_y[idx_y]]))
            zh[idx_x, idx_y] = min( [ h(np.array([dom_x[idx_x], dom_y[idx_y]]), obstacles[0]), h(np.array([dom_x[idx_x], dom_y[idx_y]]), obstacles[1]), h(np.array([dom_x[idx_x], dom_y[idx_y]]), obstacles[2]), h(np.array([dom_x[idx_x], dom_y[idx_y]]), obstacles[3]) ] )

    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.contour3D(xx, yy, zz, 25, cmap='viridis', edgecolor='none')
    ax1.contour3D(xx, yy, zh, 25, cmap='viridis', edgecolor='none')
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
