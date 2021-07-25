import tools_etc
import tools_cmd_vel
import cylinder_laser_scan_align
import cylinder_laser_scan_distance


def convex_rotate(angle_90, angle_110):
    cylinder_laser_scan_align.keep_align()
    cylinder_laser_scan_distance.keep_distance(1)

    a, c = angle_110, angle_90
    b = tools_etc.triangle(20, a, c)
    squeezed = tools_etc.squeeze_triangle(a, b, c)
    x, y = squeezed[2], squeezed[1]
    tools_cmd_vel.move_relative_rotate(x-0.05, y-0.15)


def concave_rotate(angle_90, angle_110):
    cylinder_laser_scan_align.keep_align()
    cylinder_laser_scan_distance.keep_distance(1)

    a, c = angle_110, angle_90
    b = tools_etc.triangle(20, a, c)
    squeezed = tools_etc.squeeze_triangle(a, b, c)
    x, y = squeezed[2], squeezed[1]
    tools_cmd_vel.move_relative_rotate(x, y-0.15)
