import numpy as np
import matplotlib.pyplot as plt

xs = np.arange(0, 2, 0.01)
ys = [x**2 - 2*x for x in xs]

def y(x):
    return x - 2.25

plt.plot(xs, ys)
plt.plot([1, 2], [y(1), y(2)])
plt.xlim(1, 2)
plt.ylim([-1.5, 1])
plt.show()