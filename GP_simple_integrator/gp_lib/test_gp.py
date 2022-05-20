import numpy as np
import matplotlib.pyplot as plt
from gp import GaussianProcess as GP

# Define Function to Fit
f = lambda x: np.sin(x)
fd = lambda x: np.cos(x)

# Initializer - Declare Size of x (1 Here) and of y (1 Here)
fitter = GP(1, 1)

# Add Samples
x = np.arange(-1, 3, 1)
y = f(x)
for idx in range(0, x.shape[0]):
  fitter.add_sample(x[idx], y[idx])

# Fit Hyperparameters - Not Mandatory (Very High Computational Complexity)
print('Initial Hyperparams: ' + str(fitter.get_hyperparams()))
fitter.optimize_hyperparameters()
print('Post Optimization Hyperparams: ' + str(fitter.get_hyperparams()))

# Train
print('Training With ' + str(fitter.get_nsample()) + ' Samples')
fitter.train()

# Get Inference & Variance
dom = np.arange(-2, 4, 0.1)
yy = np.empty(dom.shape)
yv = np.empty(dom.shape)
for idx in range(dom.shape[0]):
  yy[idx] = fitter.posterior_mean(dom[idx])
  yv[idx] = fitter.posterior_variance(dom[idx])

fig = plt.figure()
plt.plot(dom, yy, label='Gaussian Process Inference')
plt.plot(dom, yy + yv, ':', label='Upper Bound Variance')
plt.plot(dom, yy - yv, ':', label='Lower Bound Variance')
plt.plot(dom, f(dom), label='Real Function')
plt.plot(x, y, '.', label='Sampled Points')
plt.grid()
plt.legend()

# Get Derivative
yd = np.empty(dom.shape)
for idx in range(dom.shape[0]):
  yd[idx] = fitter.posterior_dxmean(dom[idx])

fig = plt.figure()
plt.plot(dom, yd, label='Gaussian Process Derivative')
plt.plot(dom, fd(dom), label='Real Function')
plt.grid()
plt.legend()

plt.show()
