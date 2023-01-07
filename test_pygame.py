import math
import pygame
import sys
import copy
from pygame.locals import *
from scipy import interpolate
import numpy as np
import random
from time import sleep

FPS = 60
FPS_CLOCK = pygame.time.Clock()
WIN_WIDTH, WIN_HEIGHT = 640, 480

ROBOT_X = 10
ROBOT_Y = 10
ROBOT_TH = 0 # yaw angle, the direction the robot actually facing
ROBOT_DELTA = 0 # steering angle, the direction that the front wheels are facing (Ackermann drive)

ROBOT_W = 30
ROBOT_L = 60
ROBOT_VEL = 0
WHEEL_W = 5
WHEEL_L = 10
WHEEL_POS = [[10,15],[10,-15],[-10,15],[-10,-15]]

ROBOT_POINTS = [
    [+ROBOT_W / 2, +ROBOT_L / 2],
    [+ROBOT_W / 2, -ROBOT_L / 2],
    [-ROBOT_W / 2, -ROBOT_L / 2],
    [-ROBOT_W / 2, +ROBOT_L / 2]
]
WHEEL_POINTS = [
    [+WHEEL_W / 2, +WHEEL_L / 2],
    [+WHEEL_W / 2, -WHEEL_L / 2],
    [-WHEEL_W / 2, -WHEEL_L / 2],
    [-WHEEL_W / 2, +WHEEL_L / 2]
]
RENDER_MARGIN = 5
old_center_points = []

PARKING_LOTS_RENDER = []
PARKING_LOTS_CENTER = []
PARKING_LOT_W = 70
PARKING_LOT_L = 40
PARKING_LOT_START_X = 300 + RENDER_MARGIN*10
PARKING_LOT_START_Y = 10 + RENDER_MARGIN*10
PARKING_LOT_N = 5

screen = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))
pygame.init()

def d2r(degree):
    return degree / 180 * math.pi
def r2d(radian):
    return radian / math.pi * 180
# https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python
def rotate_1_point(origin, point, angle, counterclockwise=False):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in degrees.
    """
    ox, oy = origin
    px, py = point

    rotate_angle = d2r(angle if counterclockwise else -angle)
    qx = ox + math.cos(rotate_angle) * (px - ox) - math.sin(rotate_angle) * (py - oy)
    qy = oy + math.sin(rotate_angle) * (px - ox) + math.cos(rotate_angle) * (py - oy)
    return qx, qy
def rotate(center, points, angle, ccw=False):
    return [rotate_1_point(center, pt, angle, ccw) for pt in points]
def arrow(screen, lcolor, tricolor, start, end, trirad, thickness=2, arrow_scale=10):
    pygame.draw.line(screen, lcolor, start, end, thickness)
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi/2
    pygame.draw.polygon(screen, tricolor, ((end[0] + trirad * math.sin(rotation) * arrow_scale,
                                        end[1] + trirad * math.cos(rotation) * arrow_scale),
                                       (end[0] + trirad * math.sin(rotation - d2r(120)) * arrow_scale,
                                        end[1] + trirad * math.cos(rotation - d2r(120)) * arrow_scale),
                                       (end[0] + trirad * math.sin(rotation + d2r(120)) * arrow_scale,
                                        end[1] + trirad * math.cos(rotation + d2r(120)) * arrow_scale)))
def arrow_to(start, angle, line_color, head_color, thickness=2, arrow_scale=10):
    arrow(screen, line_color, head_color, start, (start[0] + math.sin(d2r(angle))*50, start[1]+math.cos(d2r(angle))*50), d2r(40), thickness, arrow_scale)

def render_one_frame():
    global ROBOT_POINTS, ROBOT_TH, WHEEL_POS, ROBOT_Y, ROBOT_DELTA, ROBOT_X, old_center_points
    
    # Draw the chassis.
    robot_points = [(elem[0] + ROBOT_X, elem[1] + ROBOT_Y) for elem in ROBOT_POINTS]
    rotated_robot = rotate((ROBOT_X, ROBOT_Y), robot_points, ROBOT_TH, True)
    for idx, elem in enumerate(rotated_robot):
        rotated_robot[idx] = (elem[0] + 10*RENDER_MARGIN, elem[1] + 10*RENDER_MARGIN)
    pygame.draw.polygon(screen, (255, 255, 255), rotated_robot, 2)
    
    # Draw the center and the arrow.
    actual_center = (ROBOT_X + 10*RENDER_MARGIN, ROBOT_Y + 10*RENDER_MARGIN)
    old_center_points.append(actual_center)
    for pt in old_center_points:
        pygame.draw.circle(screen, (0, 255, 255), pt, 2)
    arrow(screen, (255, 255, 255), (255, 255, 255), actual_center,
        (actual_center[0] + math.sin(d2r(ROBOT_TH)) * 50, actual_center[1] + math.cos(d2r(ROBOT_TH)) * 50),
        d2r(40), 2, 8)
    
    # Draw the wheels.
    rotated_wheel_center = rotate((0, 0), WHEEL_POS, ROBOT_TH, True)
    for i,wheel in enumerate(rotated_wheel_center):
        if i == 1 or i == 3:
            rotated_wheel = rotate((0, 0), WHEEL_POINTS, ROBOT_DELTA+ROBOT_TH, True)
        else:
            rotated_wheel = rotate((0, 0), WHEEL_POINTS, ROBOT_TH, True)
        for j,rt_wheel in enumerate(rotated_wheel):
            rotated_wheel[j] = (
                ROBOT_X + wheel[0] + 10*RENDER_MARGIN + rt_wheel[0],
                ROBOT_Y + wheel[1] + 10*RENDER_MARGIN + rt_wheel[1]
            )
        pygame.draw.polygon(screen, (255, 0, 0), rotated_wheel)

def move(accel, steering_angle):
    global ROBOT_X, ROBOT_Y, ROBOT_VEL, ROBOT_TH, ROBOT_DELTA
    x = ROBOT_VEL * math.cos(d2r(ROBOT_TH))
    y = ROBOT_VEL * math.sin(d2r(ROBOT_TH))
    v = accel
    th = ROBOT_VEL * math.tan(d2r(steering_angle)) / ROBOT_L
    delta = steering_angle
    return x,y,v,th,delta
def update(pack):
    global ROBOT_X, ROBOT_Y, ROBOT_VEL, ROBOT_TH, ROBOT_DELTA
    x,y,v,th,delta = pack
    ROBOT_X += 0.2 * x
    ROBOT_Y += 0.2 * y
    ROBOT_VEL += 0.2 * v
    ROBOT_TH += 0.2 * th
    ROBOT_DELTA = delta
def move_and_update(accel, steering_angle):
    update(move(accel, steering_angle))
def wrap(angle):
    return r2d(math.remainder(d2r(angle), 2*math.pi))
def init_parking_lots():
    global PARKING_LOTS_RENDER, PARKING_LOTS_CENTER, PARKING_LOT_W, PARKING_LOT_L, PARKING_LOT_START_X, PARKING_LOT_START_Y, PARKING_LOT_N
    x = PARKING_LOT_START_X
    y = PARKING_LOT_START_Y
    for i in range(PARKING_LOT_N):
        PARKING_LOTS_RENDER.append(( x + PARKING_LOT_W/2, y + PARKING_LOT_L/2 ))
        PARKING_LOTS_CENTER.append(( x + PARKING_LOT_W, y + PARKING_LOT_L ))
        y += (PARKING_LOT_L+10)
def redraw_parking_lots():
    global PARKING_LOTS_RENDER, PARKING_LOTS_CENTER, PARKING_LOT_W, PARKING_LOT_L, PARKING_LOT_START_X, PARKING_LOT_START_Y, PARKING_LOT_N
    for i in range(5):
        pygame.draw.rect(screen, (255, 255, 255), (PARKING_LOTS_RENDER[i][0], PARKING_LOTS_RENDER[i][1], 70, 40), 2)
        pygame.draw.circle(screen, (255, 0, 255), (PARKING_LOTS_CENTER[i][0], PARKING_LOTS_CENTER[i][1]), 5)
phi_ref = 0

pos_err = [0, 0] # positional error
pos_integral = 0
pos_deriv = 0
last_pos_err = [0, 0]

ori_err = 0 # orientational error
ori_integral = 0
ori_deriv = 0
last_ori_err = 0

already_close_to_targ=False

angle_tolerance=5
dist_tolerance=3

def find_min_angle (targetHeading, currentHeading):
    turnAngle = targetHeading - currentHeading
    if turnAngle > 180 or turnAngle < -180 :
        turnAngle = -1 * np.sign(turnAngle) * (360 - abs(turnAngle))
    return turnAngle
def move_to_point_each_frame(coord, angle_desired=None, curve_scale=2, back=False):
    global phi_ref, pos_err, ROBOT_X, ROBOT_Y, ROBOT_TH, ROBOT_DELTA, last_pos_err, pos_integral, pos_deriv, ori_integral, ori_err, ori_deriv, last_ori_err, dyn_x, dyn_y, already_close_to_targ,angle_tolerance,dist_tolerance
    if abs(math.dist((ROBOT_X, ROBOT_Y), coord)) <= dist_tolerance and abs(wrap(ROBOT_TH - (phi_ref if angle_desired == None else angle_desired))) <= angle_tolerance:# and abs((180-ROBOT_TH) - (180-phi_ref)) <= 5:
       pass
    else:
       phi_ref = r2d(math.atan2(coord[1]-ROBOT_Y, coord[0]-ROBOT_X))
       
       if back:
            phi_ref = 180 + phi_ref
       
       # ===========
       pos_err[0] = coord[0] - ROBOT_X
       pos_err[1] = coord[1] - ROBOT_Y
       dist = math.sqrt(pos_err[0]**2 + pos_err[1]**2)
       ori_err = wrap(phi_ref - ROBOT_TH)
       
       if angle_desired != None:
            alpha = wrap(phi_ref - angle_desired)
            beta = r2d(math.atan(curve_scale / dist))
            if alpha < 0:
                beta = -beta
            if abs(alpha) < abs(beta):
                ori_err = wrap(find_min_angle(phi_ref, ROBOT_TH) + alpha)
            else:
                ori_err = wrap(find_min_angle(phi_ref, ROBOT_TH) + beta)
       
       #if ori_err < 0:
       #     ori_err += 2 * math.pi
       
       pos_integral = pos_integral + dist if dist >= dist_tolerance else 0
       pos_deriv = dist - math.sqrt(last_pos_err[0]**2 + last_pos_err[1]**2)
       ori_integral = ori_integral + ori_err if ori_err >= angle_tolerance else 0
       ori_deriv = ori_err - last_ori_err
       # ===========
       
       v = 3 * dist + .000*pos_integral + 0*pos_deriv
       delta = (5*ori_err + .00*ori_integral + 0*ori_deriv) #23.515
       
       if dist<=15:
            already_close_to_targ = True
       if already_close_to_targ:
            v *= math.copysign(1, math.cos(d2r(ori_err)))
            ori_err = find_min_angle(phi_ref if angle_desired == None else angle_desired, ROBOT_TH)
            delta = 15*r2d(math.atan(math.tan(d2r(ori_err))))# + .001*ori_integral + 20*ori_deriv
            #print(v)
            
       delta = delta if abs(delta) < 200 else 200 * math.copysign(1, delta)
       v = v if abs(v) < 150 else 150 * math.copysign(1, v)
       
       if back:
            v *= -1
       
       #if (angle_desired == None and abs(ROBOT_TH - phi_ref) <= 10) or (angle_desired != None and abs(ROBOT_TH - angle_desired) <= 30):
       #     v /= 1.1
       
       if v > (150 - np.abs(delta)):
            v = 150 - np.abs(delta)
       
       # manual simulation here
       dq = [v*math.cos(d2r(ROBOT_TH)), v*math.sin(d2r(ROBOT_TH)), delta]
       noise = 0.001
       ROBOT_X = ROBOT_X + (1/FPS)*dq[0] + random.randrange(1,3)*noise
       ROBOT_Y = ROBOT_Y + (1/FPS)*dq[1] + random.randrange(1,3)*noise
       ROBOT_TH = wrap(ROBOT_TH + (1/FPS)*dq[2] + random.randrange(1,3)*noise)
       #ROBOT_DELTA = wrap(delta)
       
       actual_center = (ROBOT_X + 10*RENDER_MARGIN, ROBOT_Y + 10*RENDER_MARGIN)
       #arrow_to(actual_center, ROBOT_DELTA, (0, 255, 255), (0, 255, 255))
       if angle_desired == None:
            arrow_to(actual_center, phi_ref, (255, 0, 0), (255, 0, 0), 2, 5)
       else:
            arrow_to(actual_center, angle_desired, (255, 0, 0), (255, 0, 0), 2, 5)
       print(abs(math.dist((ROBOT_X, ROBOT_Y), coord)), abs(wrap(ROBOT_TH - (phi_ref if angle_desired == None else angle_desired))))
       #print(ROBOT_X, ROBOT_Y, ROBOT_TH, phi_ref)
       #print(delta, ori_err, ori_integral, ori_deriv)
       #print(delta, v, abs(math.dist((ROBOT_X, ROBOT_Y), coord)), abs(ROBOT_TH - phi_ref))
       
       last_pos_err = pos_err
       last_ori_err = ori_err
# Game loop.
dyn_x=ROBOT_X
dyn_y=-1
dyn_th=-1
TARG_PARKING_LOT = 2

def spline_out(x):
    global ROBOT_X, ROBOT_Y, PARKING_LOTS_CENTER, TARG_PARKING_LOT
    x_pts = [0, PARKING_LOTS_CENTER[TARG_PARKING_LOT][0]]
    y_pts = [0, PARKING_LOTS_CENTER[TARG_PARKING_LOT][1]]
    tck = interpolate.splrep(x_pts, y_pts, k =1)
    return interpolate.splev(x, tck)

init_parking_lots()
while True:
    screen.fill((0, 0, 0))
    redraw_parking_lots()
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    # Update.
    pygame.draw.circle(screen, (255, 0, 255), (dyn_x+10*RENDER_MARGIN, dyn_y+10*RENDER_MARGIN), 5)
    dyn_x += .1
    #print(PARKING_LOTS_CENTER[0])
    move_to_point_each_frame((420 - 10*RENDER_MARGIN, 100 - 10*RENDER_MARGIN), 90, back=True)
    
    #dyn_y = dyn_x * 1.1
    #dyn_th += 1
    #print((dyn_x, dyn_y))
    render_one_frame()
    # Draw.
    pygame.display.update()
    FPS_CLOCK.tick(FPS)