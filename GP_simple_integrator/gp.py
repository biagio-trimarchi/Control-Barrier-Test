import numpy as np
import math as mt
from scipy.optimize import minimize
from scipy.optimize import Bounds

class Parameters:
  def __init__(self, n, m):
    self.order_input = n
    self.order_output = m
    self.gp_L = np.empty((0))
    self.gp_alpha = np.empty((0))
    self.gp_x = np.empty((n,0))
    self.gp_y = np.empty((m,0))
    self.s = 1e-3
    self.tau = np.ones((n,))*1

class GaussianProcess:
  def __init__(self, n, m):
    self.params = Parameters(n, m)

  def k(self, x1, x2, tau):
    if len(x1.shape) == 1:
      x1 = x1.reshape((self.params.order_input, 1))

    if len(x2.shape) == 1:
      x2 = x2.reshape((self.params.order_input, 1))

    n1 = x1.shape[1]
    n2 = x2.shape[1]
    kk = np.empty((n1, n2))

    for ii in range(0, n1):
      for jj in range(0, n2):
        for idx in range(0, self.params.order_input):
          d = (x1[idx,ii]-x2[idx,jj])**2
          if idx == 0:
            kk[ii,jj] = np.exp(-d/(2*tau[idx]**2))
          else:
            kk[ii,jj] = kk[ii,jj]*np.exp(-d/(2*tau[idx]**2))

    return kk

  def kdt(self, x1, x2, xd, tau):
    if len(x1.shape) == 1:
      x1 = x1.reshape((self.params.order_input, 1))
      xd = xd.reshape((self.params.order_input, 1))

    if len(x2.shape) == 1:
      x2 = x2.reshape((self.params.order_input, 1))

    n1 = x1.shape[1]
    n2 = x2.shape[1]
    kk = np.zeros((n1, n2))

    for ii in range(0, n1):
      for jj in range(0, n2):
        for idx in range(0, self.params.order_input):
          d = (x1[idx,ii]-x2[idx,jj])
          if idx == 0:
            c = np.exp(-d*d/(2*tau[idx]**2))
          else:
            c = c*np.exp(-d*d/(2*tau[idx]**2))

        for idx in range(0, self.params.order_input):
          d = x1[idx,ii]-x2[idx,jj]
          kk[ii,jj] = kk[ii,jj] - c*d*xd[idx,ii]/(tau[idx]**2)

    return kk

  def kdx(self, x1, x2, tau):
    if len(x1.shape) == 1:
      x1 = x1.reshape((self.params.order_input, 1))

    if len(x2.shape) == 1:
      x2 = x2.reshape((self.params.order_input, 1))

    n1 = x1.shape[1]
    n2 = x2.shape[1]
    kk = np.zeros((n1, n2, self.params.order_input))
    tau_ = np.zeros(tau.shape)

    for ii in range(0, tau.shape[0]):
        tau_[ii] = 1/(tau[ii]**2)

    for ii in range(0, n1):
      for jj in range(0, n2):
        for idx in range(0, self.params.order_input):
          d = (x1[idx,ii]-x2[idx,jj])
          if idx == 0:
            c = np.exp(-d*d/(2*tau[idx]**2))
          else:
            c = c*np.exp(-d*d/(2*tau[idx]**2))

        d = x1[:,ii]-x2[:,jj]
        
        kk[ii,jj,:] = -c*(d*tau_)

    return kk

  def train(self):
    kk = self.k(self.params.gp_x, self.params.gp_x, self.params.tau)
    self.params.gp_L = np.linalg.cholesky(kk + self.params.s*np.eye(kk.shape[0]))
    self.params.gp_alpha = np.linalg.solve(np.transpose(self.params.gp_L), np.linalg.solve(self.params.gp_L, self.params.gp_y.transpose()))

  def add_sample(self, x, y):
    x = x.reshape((self.params.gp_x.shape[0],1))
    y = y.reshape((self.params.gp_y.shape[0],1))
    self.params.gp_x = np.append(self.params.gp_x, x, axis=1)
    self.params.gp_y = np.append(self.params.gp_y, y, axis=1)

  def posterior_mean(self, x):
    x = x.reshape(self.params.gp_x[:,0].shape)
    kk = self.k(x, self.params.gp_x, self.params.tau)
    return kk@self.params.gp_alpha

  def posterior_dtmean(self, x, xd):
    x = x.reshape(self.params.gp_x[:,0].shape)
    xd = xd.reshape(self.params.gp_x[:,0].shape)
    kkd = self.kdt(x, self.params.gp_x, xd, self.params.tau)
    return kkd@self.params.gp_alpha

  def posterior_dxmean(self, x):
    x = x.reshape(self.params.gp_x[:,0].shape)
    kkd = self.kdx(x, self.params.gp_x, self.params.tau)
    xd = np.zeros((kkd.shape[2], 1))
    for ii in range(0,kkd.shape[2]):
      xd[ii,:] = kkd[:,:,ii]@self.params.gp_alpha
    return xd

  def posterior_variance(self, x):
    x = x.reshape(self.params.gp_x[:,0].shape)
    kk = self.k(x, self.params.gp_x, self.params.tau)
    kk_ = self.k(x, x, self.params.tau)
    alpha = np.linalg.solve(np.transpose(self.params.gp_L), np.linalg.solve(self.params.gp_L, np.transpose(kk)))
    return kk_ - kk@alpha
  
  def posterior_dxvariance(self, x):
    x = x.reshape(self.params.gp_x[:,0].shape)
    kk = self.k(x, self.params.gp_x, self.params.tau)
    alpha = np.linalg.solve(np.transpose(self.params.gp_L), np.linalg.solve(self.params.gp_L, np.transpose(kk)))
    kkd = kkd = self.kdx(x, self.params.gp_x, self.params.tau)
    
    dkk_ = np.zeros((kkd.shape[2], 1))
    for ii in range(0, kkd.shape[2]):
      dkk_[ii, :] = 2*alpha.T@kkd[:, :, ii].T
    return dkk_

  def optimize_hyperparameters(self):
    print("Inital Params: " + str(self.params.tau))
    self.params.tau = minimize(self.ll, self.params.tau, method='L-BFGS-B').x
    print("Post Opt Params: " + str(self.params.tau))

  def ll(self, x):
    kk = self.k(self.params.gp_x, self.params.gp_x, np.array([x]).reshape((self.params.order_input,1)))
    L = np.linalg.cholesky(kk + self.params.s*np.identity(kk.shape[0]))
    alpha = np.linalg.solve(np.transpose(L), np.linalg.solve(L, self.params.gp_y.transpose()))
    return (np.matmul(np.transpose(self.params.gp_y.transpose()), alpha) + mt.log(abs(np.linalg.det(kk +  self.params.s*np.identity(kk.shape[0]))))).ravel()

  def get_nsample(self):
    return self.params.gp_x.shape[1]

  def get_hyperparams(self):
    return self.params.tau

  def set_noisepw(self, s):
    self.params.s = s