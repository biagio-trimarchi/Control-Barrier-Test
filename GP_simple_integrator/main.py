import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gp import GaussianProcess as GP
from scipy.integrate import ode
from cvxopt import solvers, matrix

def h(x, obstacle):
    return np.linalg.norm(x-obstacle['Position']) - obstacle['Radius']

def deltah(x, obstacle):
    return 2*(x - obstacle['Position'])

def dynamic(x, u):
    return u

def simulation(t, x, xgoal, obstacles):
    alpha = 0.3
    K = 1
    v = -K*(x- xgoal)

    # QUADRATIC PROGRAMMING
    A = matrix( -np.block([
                [deltah(x, obstacles[0])],
                [deltah(x, obstacles[1])]
    ]) )

    b = matrix( alpha*np.block([ [h(x, obstacles[0])],
                         [h(x, obstacles[1])] 
                    ]) )
    
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
    alpha = 10              # K-function
    dt = 0.01               # time-step
    dSafe = 0.2             # Safety radius

    # OBSTACLES
    obstacles = []
    obstacles.append({'Position': np.array([1, 2]), 'Radius': 0.5})
    obstacles.append({'Position': np.array([2.5, 3]), 'Radius': 0.5})

    # QUADRATRIC PROGRAMMING
    solvers.options['show_progress'] = False

    # CONTROL
    x = x0.reshape((d, 1))
    # print(x)
    # x = np.append(x, x0.reshape((d, 1)), 1)
    # print(x)

    f = lambda t,x : simulation(t, x, xgoal, obstacles)
    r = ode(f).set_integrator('dopri5')
    r.set_initial_value(x0, 0)

    while r.successful() and np.linalg.norm(x[:, -1] - xgoal) > 0.1:
        r.integrate(r.t+dt)
        x = np.append(x, (r.y).reshape((d,1)), 1)

    ### PLOTS
    circle1 = plt.Circle(obstacles[0]['Position'], obstacles[0]['Radius'], color='r')
    circle2 = plt.Circle(obstacles[1]['Position'], obstacles[1]['Radius'], color='r')

    fig, ax = plt.subplots()

    ax.add_patch(circle1)
    ax.add_patch(circle2)

    plt.plot(x[0,:], x[1,:])

    ax.set_xlim((-1, 4))
    ax.set_ylim((-1, 6))
    plt.gca().set_aspect('equal', adjustable='box')

    plt.show()


if __name__ == '__main__':
    main()