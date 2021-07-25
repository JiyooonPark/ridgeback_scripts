import math
'''
triangle calculator
https://www.calculator.net/triangle-calculator.html?vc=&vx=1&vy=&va=20&vz=1.5&vb=&angleunits=d&x=77&y=38
'''


def average(range, degree, interval=5):
    length = len(range)
    one = length/180
    # print(int(degree-interval)*one)
    return (range[int((degree-interval)*one)]+range[int(degree*one)] + range[int((degree+interval)*one)])/3


def triangle(theta, a, c):
    theta = math.radians(theta)
    b = math.sqrt((c*math.sin(theta))**2 + ((a-c*math.cos(theta))**2))
    # print("b is :", b)
    return b

# calculates the difference of angles


def diff(angle1, angle2):
    print("difference of : {:.4} and {:.4} is {:.4}".format(
        angle1, angle2, angle1-angle2))
    return angle1-angle2


def rad(degree):
    return math.radians(degree)


def squeeze_triangle(a, b, c):
    # return b, (b**2)/a, (b*c)/a
    return b, b*math.sin(rad(20)), b*math.cos(rad(20))


def two_edge_one_angle(a, c, angle):
    # outputs the other angle
    b = triangle(angle, a, c)
    angle = rad(angle)
    return math.degrees(math.acos((a-c*math.cos(angle))/b))


if __name__ == "__main__":

    c = 1
    a = 1.5
    theta = 20

    b = triangle(20, a, c)
    print(two_edge_one_angle(a, c, 20))
    print(squeeze_triangle(a, b, c))
    small_triangle = squeeze_triangle(a, b, c)
