import numpy as np
import matplotlib.pyplot as plt
from gp import GaussianProcess as GP

# Define Function to Fit
f = lambda x,y: np.sin(np.sqrt(x ** 2 + y ** 2))

# Initializer - Declare Size of x (1 Here) and of y (1 Here)
fitter = GP(2, 1)

# Sample From 3D Function
dom_x_ = np.arange(-5, 5, 1)
dom_y_ = np.arange(-5, 5, 1)

for idx_x in range(dom_x_.shape[0]):
  for idx_y in range(dom_y_.shape[0]):
      fitter.add_sample(np.array([dom_x_[idx_x], dom_y_[idx_y]]), f(dom_x_[idx_x], dom_y_[idx_y]))
  
# Fit Hyperparameters - Not Mandatory (Very High Computational Complexity)
print('Initial Hyperparams: ' + str(fitter.get_hyperparams()))
fitter.optimize_hyperparameters()
print('Post Optimization Hyperparams: ' + str(fitter.get_hyperparams()))

# Train
print('Training With ' + str(fitter.get_nsample()) + ' Samples')
fitter.train()

# Get Inference & Variance
dom_x = np.arange(-5, 5, 0.2)
dom_y = np.arange(-5, 5, 0.2)
xx, yy = np.meshgrid(dom_x, dom_y)
zz = np.empty(xx.shape)
zv = np.empty(xx.shape)
for idx_x in range(dom_x.shape[0]):
  for idx_y in range(dom_y.shape[0]):
      zz[idx_x, idx_y] = fitter.posterior_mean(np.array([dom_x[idx_x], dom_y[idx_y]]))
      zv[idx_x, idx_y] = fitter.posterior_variance(np.array([dom_x[idx_x], dom_y[idx_y]]))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.contour3D(xx, yy, zz, 25, cmap='viridis')
ax.contour3D(xx, yy, f(xx, yy), 25, cmap='binary')
plt.grid()

# Get Derivative
zd = np.empty((xx.shape[0], xx.shape[1], 2))
for idx_x in range(dom_x.shape[0]):
  for idx_y in range(dom_y.shape[0]):
      zd[idx_x, idx_y, :] = (fitter.posterior_dxmean(np.array([dom_x[idx_x], dom_y[idx_y]]))).reshape((2,))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.contour3D(xx, yy, zd[:,:,0], 25, cmap='viridis') # Plot Just Along x Axis
plt.grid()

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.contour3D(xx, yy, zd[:,:,1], 25, cmap='viridis') # Plot Just Along y Axis
plt.grid()
plt.show()

