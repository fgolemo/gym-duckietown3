import numpy as np


def get_point_from_circle_distribution(x_center, y_center, radius):
    a = np.random.uniform()
    b = np.random.uniform()
    if b < a:
        b, a = a, b

    x = b * radius * np.cos(2 * np.pi * a / b) + x_center
    y = b * radius * np.sin(2 * np.pi * a / b) + y_center

    return x, y
