# Python program to find equation of a plane
# passing through given 3 points.
 
# Function to find equation of plane.
# https://www.geeksforgeeks.org/program-to-find-equation-of-a-plane-passing-through-3-points/

def equation_plane(x1, y1, z1, x2, y2, z2, x3, y3, z3):
     
    a1 = x2 - x1
    b1 = y2 - y1
    c1 = z2 - z1
    a2 = x3 - x1
    b2 = y3 - y1
    c2 = z3 - z1
    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = (- a * x1 - b * y1 - c * z1)
    print ("equation of plane is ",)
    print (a, "x +",)
    print (b, "y +",)
    print (c, "z +")
    print (d, "= 0.")
    return a, b, c, d
 
# Driver Code

x1, y1, z1 = 0, 0, 0
x2, y2, z2 = 0, 1, 0
x3, y3, z3 = 1, 0, 0
a, b, c, d = equation_plane(x1, y1, z1, x2, y2, z2, x3, y3, z3)

import numpy as np
import math
u1 = b/math.sqrt(a**2+b**2+c**2)
u2 = -a/math.sqrt(a**2+b**2+c**2)
sin_th = math.sqrt((a**2+b**2)/(a**2+b**2+c**2))
cos_th = c/math.sqrt(a**2+b**2+c**2)
T = np.array([
    [cos_th+(u1**2)*(1-cos_th), u1*u2*(1-cos_th), u2*sin_th],
    [u1*u2*(1-cos_th), cos_th+(u2**2)*(1-cos_th), -u1*sin_th],
    [-u2*sin_th, u1*sin_th, cos_th]
])
now = np.dot(T, np.array([-1, 2, 1]))
print(now)

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

x = [x1, x2, x3]
y = [y1, y2, y3]
X,Y = np.meshgrid(x,y)
Z = X


fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection='3d')


# Plot a 3D surface
ax.plot_surface(X, Y, Z)


plt.show()