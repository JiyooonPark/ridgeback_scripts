from scipy import interpolate


def f(x):
    x_points = [0, 1, 2, 3, 4, 5]
    y_points = [1.2, 1.4, 2.2, 3.9, 5.8, 7.7]

    tck = interpolate.splrep(x_points, y_points)
    return interpolate.splev(x, tck)


x = [0.1 * x for x in range(0, 50)]
y = [f(xx) for xx in x]
print(y)
print(type(x[0]))
