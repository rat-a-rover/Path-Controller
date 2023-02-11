import numpy as np

class RLS(object):
    def __init__(self, mu, eps):
        self.R = 1/eps
        self.mu = mu
        self.wt = 0.0

    def update(self, x, x_star):
        y = x * self.wt
        self.R = 1 / self.mu * (self.R - (self.R**2  * x**2) \
                                       / (self.mu + x**2 * self.R))
        e = (x_star - y)
        self.wt += self.R * x * e
        return y, e
         
        
