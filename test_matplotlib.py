import math
import pygame
import sys
import copy
from pygame.locals import *
from scipy import interpolate
import numpy as np
import random
from time import sleep

sys.path.append("./PythonRobotics-master")
sys.path.append("./PythonRobotics-master/Control/move_to_pose")

def d2r(degree):
    return degree / 180 * math.pi
def r2d(radian):
    return radian / math.pi * 180
def truncated_remainder(dividend, divisor):
    divided_number = dividend / divisor
    divided_number = \
        -int(-divided_number) if divided_number < 0 else int(divided_number)

    remainder = dividend - divisor * divided_number

    return remainder
def wrap_rad(input_angle):
    pi = math.pi
    revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))

    p1 = truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(math.fabs((truncated_remainder(input_angle + pi, 2 * pi))
                                      / (2 * pi))) - 1))) * pi

    output_angle = p1 - p2

    return output_angle

from PathPlanning.VisibilityRoadMap.visibility_road_map import *
from Control.move_to_pose.move_to_pose_robot import *
from PathPlanning.CubicSpline import cubic_spline_planner
FPS = 100
# simulation parameters
dt = 1/FPS

show_animation = True

ROBOT_X = 0
ROBOT_Y = 0
ROBOT_TH = d2r(0)

X_TARG = 50
Y_TARG = 50
TH_TARG = d2r(0)
TARG_REACHED = False
MAX_LINEAR_SPEED = 40
MAX_ANGULAR_SPEED = 20
DIST_TOLERANCE = 1
ANGLE_TOLERANCE = d2r(3)

ROBOT_X_TRAJ = []
ROBOT_Y_TRAJ = []

ROBOT_W = 5
ROBOT_L = 10

class PathFinderController:
    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta
        self.last_rho = 0
        self.last_alpha = 0
        self.last_beta = 0
        self.rho_integral = 0
        self.rho_deriv = 0
        self.alpha_integral = 0
        self.alpha_deriv = 0
        self.beta_integral = 0
        self.beta_deriv = 0

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        
        self.rho_integral = (self.rho_integral + rho) if rho < DIST_TOLERANCE else 0
        self.rho_deriv = rho - self.last_rho
        
        self.alpha_integral = (self.alpha_integral + alpha) if alpha < ANGLE_TOLERANCE else 0
        self.alpha_deriv = alpha - self.last_alpha
        
        self.beta_integral = (self.beta_integral + beta) if beta < ANGLE_TOLERANCE else 0
        self.beta_deriv = beta - self.last_beta
        
        v = self.Kp_rho * rho + 0.00001 * self.rho_integral + 1 * self.rho_deriv
        w = (self.Kp_alpha * alpha + 0.00001 * self.alpha_integral + 1 * self.alpha_deriv) - (self.Kp_beta * beta + 0.0001 * self.beta_integral + 1 * self.beta_deriv)

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        
        self.last_rho = rho
        self.last_alpha = alpha
        self.last_beta = beta
        return rho, v, w

ROBOT_CONTROLLER = PathFinderController(12, 15, 3)

def move_to_pose():
    global ROBOT_X, ROBOT_Y, ROBOT_TH, X_TARG, Y_TARG, TH_TARG, TARG_REACHED, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,FPS, ROBOT_CONTROLLER
    global ROBOT_X_TRAJ, ROBOT_Y_TRAJ
    ROBOT_X_TRAJ.append(ROBOT_X)
    ROBOT_Y_TRAJ.append(ROBOT_Y)
    
    rho, linear_velocity, angular_velocity = \
        ROBOT_CONTROLLER.calc_control_command(
            X_TARG - ROBOT_X,
            Y_TARG - ROBOT_Y,
            ROBOT_TH, TH_TARG)
    
    if rho < DIST_TOLERANCE and wrap_rad(abs(TH_TARG - ROBOT_TH)) <= ANGLE_TOLERANCE:
        TARG_REACHED = True
    
    if abs(linear_velocity) > MAX_LINEAR_SPEED:
        linear_velocity = (np.sign(linear_velocity)
                        * MAX_LINEAR_SPEED)
    
    if abs(angular_velocity) > MAX_ANGULAR_SPEED:
        angular_velocity = (np.sign(angular_velocity)
                            * MAX_ANGULAR_SPEED)
    
    ROBOT_TH += angular_velocity * 1/FPS
    ROBOT_X += linear_velocity * np.cos(ROBOT_TH) * 1/FPS
    ROBOT_Y += linear_velocity * np.sin(ROBOT_TH) * 1/FPS

def rotate_1_point(origin, point, angle, counterclockwise=False):
    ox, oy = origin
    px, py = point

    rotate_angle = d2r(angle if counterclockwise else -angle)
    qx = ox + math.cos(rotate_angle) * (px - ox) - math.sin(rotate_angle) * (py - oy)
    qy = oy + math.sin(rotate_angle) * (px - ox) + math.cos(rotate_angle) * (py - oy)
    return qx, qy
def rotate(center, points, angle, ccw=False):
    return [rotate_1_point(center, pt, angle, ccw) for pt in points]

def draw_vehicle(x, y, theta, x_traj, y_traj):
    ROBOT_CENTER = (x, y)
    ROBOT_POINTS = [
        (x - ROBOT_W/2, y + ROBOT_L/2),
        (x + ROBOT_W/2, y + ROBOT_L/2),
        (x + ROBOT_W/2, y - ROBOT_L/2),
        (x - ROBOT_W/2, y - ROBOT_L/2),
        (x - ROBOT_W/2, y + ROBOT_L/2)
    ]
    ROBOT_POINTS = rotate(ROBOT_CENTER, ROBOT_POINTS, 90 - r2d(theta), False)
    #plt.plot([ROBOT_LEFT_TOP[0]], [ROBOT_LEFT_TOP[1]], 'ro')
    plt.plot([ROBOT_CENTER[0]], [ROBOT_CENTER[1]], 'bo')
    plt.arrow(x, y, np.cos(theta)*5, np.sin(theta)*5, color='g', width=0.4)
    plt.plot(*zip(*ROBOT_POINTS), 'k-')
    plt.plot(x_traj, y_traj, 'k--')

running = True
time = 0
ax = plt.subplot(111)

cur_idx = 20
OBSTACLES = [
    ObstaclePolygon(
        [12.9, 27.7, 17.4],
        [18.8, 8.6, 7.3]
    ),
    ObstaclePolygon(
        [30, 30, 35, 35],
        [60, 40, 40, 60]
    ),
    ObstaclePolygon(
        [39.8,53.9,58.1],
        [8.4,2.5,5.4]
    )
]
# ORDER of (x, y):
# (1st) --------- (4th)
# |                 |
# |                 |
# |                 |
# |                 |
# (2nd) --------- (3rd)
PARKING_LOTS = [
    [
        [43.4, 43.4, 56.6, 56.6, 43.4],
        [54.1, 46.3, 46.3, 54.1, 54.1]
    ]
]
TARG_PARKING_LOT = 0

X_TARG = (PARKING_LOTS[TARG_PARKING_LOT][0][2] + PARKING_LOTS[TARG_PARKING_LOT][0][0]) / 2
Y_TARG = (PARKING_LOTS[TARG_PARKING_LOT][1][0] + PARKING_LOTS[TARG_PARKING_LOT][1][1]) / 2

show_animation = False
x_arr, y_arr = VisibilityRoadMap(10).planning(
    0,
    0,
    X_TARG,
    Y_TARG,
    OBSTACLES
)
show_animation = True

x_arr.insert(3, 43.4)
y_arr.insert(3, 46.3)
#cyaw.append(d2r(45))

cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(x_arr, y_arr, ds=1)

cx.append(X_TARG)
cy.append(Y_TARG)
cyaw.append(TH_TARG)

def main():
    global running, time, FPS, TARG_REACHED, X_TARG, Y_TARG, TH_TARG, ROBOT_X, ROBOT_Y, ROBOT_TH, ROBOT_X_TRAJ, ROBOT_Y_TRAJ, OBSTACLES, cur_idx, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, DIST_TOLERANCE, PARKING_LOTS
    while running and time < 1000:
        time += 1/FPS
        
        if np.hypot(X_TARG - ROBOT_X, Y_TARG - ROBOT_Y) < 2:
            cur_idx += 1
        
        if cur_idx >= len(cx):
            #MAX_LINEAR_SPEED = 10
            #MAX_ANGULAR_SPEED = 5
            #DIST_TOLERANCE = 5
            print((np.hypot(X_TARG - ROBOT_X, Y_TARG - ROBOT_Y), r2d(wrap_rad(d2r(r2d(ROBOT_TH) - r2d(TH_TARG))))))
            cur_idx = len(cx) - 1
        
        X_TARG = cx[cur_idx]
        Y_TARG = cy[cur_idx]
        TH_TARG = cyaw[cur_idx]
        
        move_to_pose()
        
        if TARG_REACHED:
            cur_idx += 1
            TARG_REACHED = False
            
            if cur_idx >= len(cx):
                break
        
        
        plt.cla()
        #f.canvas.toolbar.update()
        #f.canvas.toolbar.push_current()
        plt.xlim(-20, 60)
        plt.ylim(-20, 60)
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        plt.arrow(0, 0, np.cos(0), np.sin(0), color='r', width=0.1)
        plt.arrow(X_TARG, Y_TARG, np.cos(TH_TARG), np.sin(TH_TARG), color='g', width=0.1)
        draw_vehicle(ROBOT_X, ROBOT_Y, ROBOT_TH, ROBOT_X_TRAJ, ROBOT_Y_TRAJ)
        for obstacle in OBSTACLES:
            obstacle.plot()
        for parking_lot in PARKING_LOTS:
            plt.plot(parking_lot[0], parking_lot[1], 'y-')
        plt.plot(cx, cy, 'r-')
        plt.plot(x_arr, y_arr, 'mo')
        plt.pause(1/FPS)

if __name__ == '__main__':
    main()
input("Press ENTER to continue")