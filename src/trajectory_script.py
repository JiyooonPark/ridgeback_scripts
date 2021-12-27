#!/usr/bin/env python3
import numpy as np
import time
from trajectory_planning import *
import rospy

class Iidgeback:
    def __init__(self, id, rx, ry, wall, radius=0.622):
        self.id = id
        self.r_center = [rx, ry]
        self.i_center = [rx, ry]
        self.r_radius = radius
        self.i_radius = 0.4
        self.cover_wall_amount = 0
        self.cover_point = []
        self.min_x = 0
        self.max_x = 0
        self.wall = wall

    def set_angle(self):
        self.cover_point.sort(key=lambda k:k[0], reverse=False)
        id, x1, y1 = self.cover_point[0]
        id, x2, y2 = self.cover_point[-1]

        hypot = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        angle = math.acos((x2-x1)/hypot)
        self.angle = math.pi/2 - angle
        print('--------------------angle: ',math.degrees(self.angle))
        self.set_ridgeback()

        return self.angle

    def set_ridgeback(self, hypot= 0.8):
        id, x1, y1 = self.cover_point[0]
        id, x2, y2 = self.cover_point[-1]

        direction = (y2-y1)/(x2-x1)

        if direction <= 0:
            self.r_center[0] = self.i_center[0] - hypot*math.cos(self.angle)
            self.r_center[1] = self.i_center[1] + hypot*math.sin(self.angle)
            self.direction = 'l'
            if not self.ridgeback_can_go():
                self.r_center[0] = self.i_center[0] - hypot * math.cos(self.angle)
                self.r_center[1] = self.i_center[1] - hypot * math.sin(self.angle)
                self.direction = 'r'
        else:
            self.r_center[0] = self.i_center[0] + hypot*math.cos(self.angle)
            self.r_center[1] = self.i_center[1] + hypot*math.sin(self.angle)
            self.direction = 'r'
            if not self.ridgeback_can_go():
                self.r_center[0] = self.i_center[0] + hypot * math.cos(self.angle)
                self.r_center[1] = self.i_center[1] - hypot * math.sin(self.angle)
                self.direction = 'l'
        # if not self.ridgeback_can_go():
        #     plt.show()
            # raise "Ridgeback Cannot generate Path"

    # 벽과 거리를 두기 위한 함수
    def in_limit(self, i, j):
        return False

    def can_be_generated(self, wall):
        for (id, i, j) in wall.uncovered + wall.covered:
            if self.in_limit(i, j):
                return False
        return True

    def ridgeback_can_go(self):
        for (id, i, j) in self.wall.uncovered + self.wall.covered:
            if ((i - self.r_center[0]) ** 2 + (j - self.r_center[1]) ** 2 - (self.r_radius + 0.1) ** 2) <=0:
                return False
        return True

    # 점이 원 내부에 있는지 확인하는 함수
    def in_iiwa_range(self, i, j):
        return ((i - self.i_center[0]) ** 2 + (j - self.i_center[1]) ** 2 - self.i_radius ** 2) < 0

    # 원을 plot하는 함수
    def print_map(self):
        i_circle = plt.Circle((self.i_center[0], self.i_center[1]), self.i_radius, fill=None, alpha=1, color='orange')
        r_circle = plt.Circle((self.r_center[0], self.r_center[1]), self.r_radius, fill=None, alpha=1)
        plt.gca().add_patch(i_circle)
        plt.gca().add_patch(r_circle)

    # 벽을 얼마나 커버하는지 반환하는 함수
    def cover_amount(self, wall):
        count = 0
        for (id, x, y) in wall.uncovered:
            if self.in_iiwa_range(x, y):
                count += 1
            else:
                continue
        self.cover_wall_amount = count
        return count

    # 원이 벽의 어떤 점을 커버하는지 계산하여 원의 멤버 변수에 저장
    def calc_cover_points(self, wall):
        for (id, x, y) in zip(wall.wall_id, wall.xpoints, wall.ypoints):
            if self.in_iiwa_range(x, y):
                self.cover_point.append((id, x, y))
            else:
                continue
        return self.cover_point

    def plot_direction(self):
        plt.plot([self.r_center[0], self.i_center[0]], [self.r_center[1], self.i_center[1]], 'm-')

    def print_id(self):
        plt.text(self.r_center[0], self.r_center[1], self.id, fontsize=1)

# 벽 class
class Wall:
    def __init__(self, x, y):
        self.covered = []
        self.wall_id = [k for k in range(len(x))]
        self.uncovered = list(zip(self.wall_id, x, y))
        self.xpoints = x
        self.ypoints = y


    # 벽에서 커버된 점을 추가함
    def add_covered(self, candidate):
        cx = []
        cy = []
        for (id, x, y) in candidate.calc_cover_points(self):
            if (id, x, y) in self.uncovered:
                print('(' + str(x) + ',' + str(y) + ')', end=' ')
                self.covered.append((id, x, y))
                self.uncovered.remove((id, x, y))
                cx.append(x)
                cy.append(y)
            else:
                continue
        plt.scatter(cx, cy, marker="1")
        print()
        print('length covered:', len(self.covered), '/', len(self.uncovered) + len(self.covered))

    # 모든 점이 커버되었는지 확인
    def allcovered(self):
        if len(self.uncovered) <3:
            return True
        else:
            return False

    # plot하는 함수
    def print_map(self):
        plt.plot(self.xpoints, self.ypoints)


# 초기에 원 후보를 생성
class Candidate:
    def __init__(self, wall, input_wall):
        self.wall = wall
        self.candidate = self.generate_c()
        self.input_wall = input_wall

    def generate_c(self):
        start = time.time()
        IR = []
        id = 0
        count = 1
        x_interval = generate_interval(self.wall.xpoints, count)
        y_interval = generate_interval(self.wall.ypoints, count)
        for i in range(len(x_interval)):
            for j in numpy.linspace(1, 1.3, 20):
                generated_circle = Iidgeback(id, round(x_interval[i], 3), round(y_interval[i]-0.2, 3), self.wall)
                if generated_circle.can_be_generated(wall):
                    IR.append(generated_circle)
                else:
                    continue
                id += 1

        print('points generated. Time:', time.time() - start, len(IR))
        return IR

    def delete_c(self, c):
        self.candidate.remove(c)

    def draw_candidates(self):
        for i in self.candidate:
            i.print_map()

class RidgebackCandidate:
    def __init__(self, wall, input_wall):
        self.wall = wall
        self.candidate = self.generate_c()
        self.input_wall = input_wall

    def generate_c(self):
        IR = []
        id = 0
        count = 1
        x_interval = generate_interval(self.wall.xpoints, count)
        y_interval = generate_interval(self.wall.ypoints, count)
        for i in range(len(x_interval)):
            for j in np.arange(y_interval[i] - 2, y_interval[i] - 0.2, 0.05):
                generated_circle = Iidgeback(id, round(x_interval[i], 3), round(j, 3), self.wall)
                if generated_circle.can_be_generated(wall):
                    IR.append(generated_circle)
                else:
                    continue
                id += 1
        return IR

    def draw_candidates(self):
        for i in self.candidate:
            i.print_map()

# 가장 많은 점을 커버하는
def max_coverage(wall, C):

    max_covered_points = 0
    candidates = []
    final_candidates = []

    for c in C:
        covered_points = c.cover_amount(wall)
        if covered_points >= max_covered_points:
            max_covered_points = covered_points
            candidates.append(c)
        else:
            continue

    for c in candidates:
        covered_points = c.cover_amount(wall)
        if covered_points == max_covered_points:
            final_candidates.append(c)
        else:
            continue
    selected = final_candidates[len(final_candidates) // 3]
    print("++++++++++++++++++++++", len(final_candidates))
    print("max_coverage id:", selected.id, 'covers:', max_covered_points)
    if max_covered_points == 0:
        return None
    return selected


# 실제로 다음 원을 선택하는 그리디 알고리즘
def greedy_cover_iiwa(wall, C):

    print('in greedy')

    t_start = time.time()

    steps = []
    
    while not wall.allcovered():
        max_circle = max_coverage(wall, C.candidate)
        if max_circle == None:
            return steps
        wall.add_covered(max_circle)
        C.delete_c(max_circle)
        steps.append(max_circle)
        max_circle.set_angle()
        max_circle.print_map()
        max_circle.print_id()
        max_circle.plot_direction()

    print('wall all covered')
    print("path generated with", len(steps), "circles")

    t_end = time.time()

    print('time: ', t_end - t_start)
    return steps


def generate_interval(wall, count=4):
    interval_wall = []
    for i in range(len(wall) - 1):
        interval_wall.extend(numpy.linspace(wall[i], wall[i + 1], count))
    rounded_wall = [round(x, 3) for x in interval_wall]
    return rounded_wall

if __name__ == "__main__":

    # Communication #3 : farmost drawing point
    ir_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton', Odometry, get_communication)

    file_name = 'smooth_curve10'
    input_wall = open_file(file_name, 'obj')
    print(f'Opened file {file_name}')
    x_wall, y_wall = plot_wall_draw(input_wall)

    x = generate_interval(x_wall)
    y = generate_interval(y_wall)
    wall = Wall(x, y)

    # 그리기 관련 부분
    plt.gca().set_aspect('equal', adjustable='box')

    # 후보 생성 및 그리디 알고리즘 적용
    C = Candidate( wall, input_wall)
    # C.draw_candidates()
    print('Candidates generated')
    steps = greedy_cover_iiwa(wall, C)

    min_x_list, max_x_list, path_x, path_y, path_angle = to_gazebo_cmd_format(steps)

    iiwa_range_list = to_iiwa_range(min_x_list, max_x_list)

    # Communication # 3 : position list

    from geometry_msgs.msg import PoseArray, Pose, Quaternion
    ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/trajectory', PoseArray, queue_size=10)
    message= PoseArray()
    pose_list = []
    p = Pose()
    q = Quaternion()

    for i in len(path_x):
        p.x = path_x[i]
        p.y = path_y[i]

        q = Quaternion()
        if path_angle[i][0] == 'l':
            q.x = -float(path_angle[i][2:])
        else:
            q.x = float(path_angle[i][2:])

        p.orientation = q
        pose_list.append(p)
        
    message.poses = pose_list
    ir_pub.publish(message)
    # 그리는 부분
    plt.plot(x, y, color="grey")
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
