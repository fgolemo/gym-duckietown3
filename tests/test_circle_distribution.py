import matplotlib.pyplot as plt
import numpy as np

xs=[]
ys=[]

for i in range(1000):
    a = np.random.uniform()
    b = np.random.uniform()
    if b < a:
        b, a = a, b
    R = 2.0
    xs.append(b * R * np.cos(2 * np.pi * a / b))
    ys.append(b * R * np.sin(2 * np.pi * a / b))


plt.scatter(xs, ys)
plt.show()
