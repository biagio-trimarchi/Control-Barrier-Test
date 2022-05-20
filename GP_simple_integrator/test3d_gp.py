import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gp import GaussianProcess as GP

# Initializer - Declare Size of x (1 Here) and of y (1 Here)
fitter = GP(2, 1)

# Sample From 3D Function
dom_x_ = np.arange(-5, 5, 1)
dom_y_ = np.arange(-5, 5, 1)

for idx_x in range(dom_x_.shape[0]):
  for idx_y in range(dom_y_.shape[0]):
      fitter.add_sample(np.array([dom_x_[idx_x], dom_y_[idx_y]]), np.sin(np.sqrt(dom_x_[idx_x] ** 2 + dom_y_[idx_y] ** 2)))
  
# Fit Hyperparameters - Not Mandatory (Very High Computational Complexity)
fitter.optimize_hyperparameters()

# Train
fitter.train()

# Get Inference & Variance
dom_x = np.arange(-5, 5, 0.1)
dom_y = np.arange(-5, 5, 0.1)
xx, yy = np.meshgrid(dom_x, dom_y)
zz = np.empty(xx.shape)
zv = np.empty(xx.shape)
for idx_x in range(dom_x.shape[0]):
  for idx_y in range(dom_y.shape[0]):
      zz[idx_x, idx_y] = fitter.posterior_mean(np.array([dom_x[idx_x], dom_y[idx_y]]))
      zv[idx_x, idx_y] = fitter.posterior_variance(np.array([dom_x[idx_x], dom_y[idx_y]]))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.contour3D(xx, yy, zz, 25, cmap='viridis', edgecolor='none')
ax.contour3D(xx, yy, np.sin(np.sqrt(xx ** 2 + yy ** 2)), 25,cmap='binary')

plt.grid()
plt.show()
