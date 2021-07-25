import math
import tools_cylinder

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


def detect_convex_concave(msg):
    state = 0
    state_discription = {0: 'convex', 1: 'convex->concave',
                         2: 'concave', 3: 'concave-> convex'}
    '''
    state 0: convex
    state 1: convex->concave
    state 2: concave
    state 3: concave-> convex

    usually detect 70, 90, 110
    first allign then detect how 60, 120 are. 
    if 
    {
        70 : 1.5, 90: 1, 110 : 1.5
    }
    if {
        60 : 1.6 120: 1.6 
    } then convex
    else if{
        60 : 1.6 120 : 1.3
    } then convex -> concave
    else if {
        60 : 1.3, 120 : 1.6
    } then concave -> convex
    else{
        60 : 1.3, 120 : 1.3
    } then concave
    
    '''
    thresh = 0.05

    tools_cylinder.keep_align()

    angle_70 = average(msg.ranges, 70)
    angle_90 = average(msg.ranges, 90)
    angle_110 = average(msg.ranges, 110)
    angle_60 = average(msg.ranges, 60)
    angle_120 = average(msg.ranges, 120)

    print()

    dist_60_70 = angle_60 - angle_70
    dist_110_120 = angle_110 - angle_120

    if dist_60_70 > thresh and dist_110_120 < -thresh:
        state = 0
    elif dist_60_70 > thresh and dist_110_120 > thresh:
        state = 1
    elif dist_60_70 < -thresh and dist_110_120 > thresh:
        state = 2
    else:
        state = 3
    print(state_discription[state])

    return state


if __name__ == "__main__":

    c = 1
    a = 1.5
    theta = 20

    b = triangle(20, a, c)
    print(two_edge_one_angle(a, c, 20))
    print(squeeze_triangle(a, b, c))
    small_triangle = squeeze_triangle(a, b, c)
