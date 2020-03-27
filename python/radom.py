# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.mlab as MLA
 
mu, sigma = 10, 10
x = mu + sigma*np.random.randn(5000)
 
# the histogram of the data
n, bins, patches = plt.hist(x, 20, normed=1, facecolor='blue', alpha=0.8)
 
# add a 'best fit' line
y = MLA.normpdf( bins, mu, sigma)
"""
normpdf函数原型：
        matplotlib.mlab.normpdf(x, *args)
功能：Return the normal pdf evaluated at x; args provides mu, sigma
"""
l = plt.plot(bins, y, 'g--', linewidth=3)
 
plt.xlabel('samples')
plt.ylabel('p')
plt.title(r'$Normal\ pdf\ m=10,\ \sigma=10$')
plt.axis([-30, 50, 0, 0.042])
plt.grid(True)
plt.show()
