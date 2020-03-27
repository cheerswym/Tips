import matplotlib.pyplot as plt
from scipy.stats import norm
import numpy as np

sigma_values = [0.5, 1.0, 1.5]
linestyles = ['-', '--', ':']
mu_values = [1, -1, 0.3]
x = np.linspace(-10, 10, 1000)

plt.figure(figsize=(8, 8))

composite = None
for mu, sigma, ls in zip(mu_values, sigma_values, linestyles):
    dist = norm(mu, sigma).pdf(x)
    plt.plot(x, dist, ls=ls,
             label='mu=%i, sigma=%.1f' % (mu, sigma))
    if composite is not None:
        composite += dist
    else:
        composite = dist

plt.plot(x, composite, label='composite(unweighted)')

plt.xlim(-5, 5)
plt.ylim(0, 1.85)

plt.xlabel('x')
plt.ylabel('p(x|mu,sigma)')
plt.title('Gaussian Distrubution')

plt.legend()
plt.show()
