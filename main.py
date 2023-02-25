import pymurapi as mur
import numpy as np
from math import *
import cv2 as cv
import time

auv = mur.mur_init()
x, y = 999, 999
angle = 0
x_center, y_center = 999, 999
colors = {'blue': ((130, 0, 0), (148, 255, 255)),
          'yellow': ((10, 0, 0), (30, 255, 205)),
          'green': ((60, 0, 0), (100, 255, 255))}
green, yellow = 0, 0


def clamp(v, min, max):
    if v > max:
        return max
    if v < min:
        return min
    return v


class PD(object):
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p(self, value):
        self._kp = value

    def set_d(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        out = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return out


def keep_depth(depth, P, D):
    try:
        error = auv.get_depth() - depth
        out = keep_depth.reg.process(error)
        out = clamp(out, -100, 100)
        auv.set_motor_power(2, out)
        auv.set_motor_power(3, out)
    except:
        keep_depth.reg = PD()
        keep_depth.reg.set_p(P)
        keep_depth.reg.set_d(D)


def to_180(angle_):
    if angle_ > 180.0:
        return angle_ - 360
    if angle_ < -180.0:
        return angle_ + 360
    return angle_


def keep_yaw(yaw, power, P, D):
    to_180(yaw)
    try:
        error = auv.get_yaw() - yaw
        error = to_180(error)
        out = keep_yaw.reg.process(error)
        out = clamp(out, -100, 100)
        auv.set_motor_power(0, clamp((power - out), -100, 100))
        auv.set_motor_power(1, clamp((power + out), -100, 100))
    except:
        keep_yaw.reg = PD()
        keep_yaw.reg.set_p(P)
        keep_yaw.reg.set_d(D)


def get_2color_cont(img, color1, color2):
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(img_hsv, color1[0], color1[1])
    mask2 = cv.inRange(img_hsv, color2[0], color2[1])
    mask = cv.bitwise_or(mask1, mask2)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return contours


def get_cont(img, color):
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, color[0], color[1])
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return contours


def draw_cont(img, contour):
    global x_center, y_center, x, y
    if cv.contourArea(contour) < 100:
        return
    cv.drawContours(img, [contour], 0, (0, 0, 0), 2)
    moments = cv.moments(contour)
    xm1 = moments['m10']
    xm2 = moments['m00']
    ym1 = moments['m01']
    ym2 = moments['m00']
    x = int(xm1 / xm2)
    y = int(ym1 / ym2)
    x_center = x - (320 / 2)
    y_center = y - (240 / 2)
    cv.circle(img, (x, y), 3, (0, 0, 255), -1)


def get_color(color):
    img = auv.get_image_bottom()
    cont_img = img.copy()
    for _ in colors:
        contours = get_cont(img, colors[color])
        if not contours:
            continue
        for cnt in contours:
            draw_cont(cont_img, cnt)
    cv.imshow("gen", img)
    cv.imshow("cont", cont_img)
    cv.waitKey(1)
    return contours, img


def get_2color(color1, color2):
    img = auv.get_image_bottom()
    cont_img = img.copy()
    for _ in colors:
        contours = get_2color_cont(img, colors[color1], colors[color2])
        if not contours:
            continue
        for cnt in contours:
            draw_cont(cont_img, cnt)
    cv.imshow("gen", img)
    cv.imshow("cont", cont_img)
    cv.waitKey(1)
    return contours, img


def turn(degres, depth, time_):
    timing = time.time()
    while True:
        get_color('green')
        keep_depth(depth, 70, 5)
        keep_yaw(degres, 0, 0.8, 0.5)
        if time.time() - timing > time_:
            timing = time.time()
            break


def go(degres, power, time_, depth, color):
    global angle
    timing = time.time()
    while True:
        cnt = get_color(color)
        keep_depth(depth, 50, 7)
        keep_yaw(degres, power, 0.8, 0.5)
        if time.time() - timing > time_:
            if cnt is not None:
                try:
                    angle = (angle + 180 + 180) % 360 - 180
                except:
                    pass
            timing = time.time()
            break


def depthing(depth, time_):
    timing = time.time()
    while True:
        get_color('green')
        keep_depth(depth, 20, 2.5)
        if time.time() - timing > time_:
            timing = time.time()
            break


def centralize(color1, color2, depth):
    x_center = x - (320 / 2)
    y_center = y - (240 / 2) + 22
    keep_depth(depth, 25, 2)
    try:
        get_2color(color1, color2)
        lenght = sqrt(x_center ** 2 + y_center ** 2)
        if lenght < 4.0:
            auv.set_motor_power(0, 0)
            auv.set_motor_power(1, 0)
            auv.set_motor_power(4, 0)
            return True
        outForward = centralize.regForward.process(y_center)
        outForward = clamp(outForward, -10, 10)
        outSide = centralize.regSide.process(x_center)
        outSide = clamp(outSide, -10, 10)
        auv.set_motor_power(0, -outForward)
        auv.set_motor_power(1, -outForward)
        auv.set_motor_power(4, -outSide)
    except:
        centralize.regForward = PD()
        centralize.regForward.set_p(0.3)
        centralize.regForward.set_d(0.2)

        centralize.regSide = PD()
        centralize.regSide.set_p(0.2)
        centralize.regSide.set_d(0.3)
    return False


def centralize_side(color1, color2):
    x_center = x - (320 / 2)
    try:
        get_2color(color1, color2)
        outSide = centralize.regSide.process(x_center)
        outSide = clamp(outSide, -10, 10)
        auv.set_motor_power(4, -outSide)
    except:
        centralize.regSide = PD()
        centralize.regSide.set_p(0.2)
        centralize.regSide.set_d(0.3)
    return False


def area_2color_shape(color1, color2, img):
    contours = get_2color_cont(img, colors[color1], colors[color2])
    if contours:
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < 200:
                continue
            (circle_x, circle_y), circle_radius = cv.minEnclosingCircle(cnt)
            circle_area = circle_radius ** 2 * pi - 0.5
            rectangle = cv.minAreaRect(cnt)
            box = cv.boxPoints(rectangle)
            box = np.int0(box)
            rectangle_area = cv.contourArea(box)
            rect_w, rect_h = rectangle[1][0], rectangle[1][1]
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)
            try:
                triangle = cv.minEnclosingTriangle(cnt)[1]
                triangle = np.int0(triangle)
                triangle_area = cv.contourArea(triangle)
            except:
                triangle_area = 0
            shapes_areas = {
                'circle': circle_area,
                'rectangle' if aspect_ratio > 1.5 else 'square': rectangle_area,
                'triangle': triangle_area,
            }
            diffs = {
                name: abs(area - shapes_areas[name]) for name in shapes_areas
            }
            shape_name = min(diffs, key=diffs.get)
            return shape_name, area


def area_shape(color1, img):
    contours = get_cont(img, colors[color1])
    if contours:
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < 200:
                continue
            (circle_x, circle_y), circle_radius = cv.minEnclosingCircle(cnt)
            circle_area = circle_radius ** 2 * pi - 0.5
            rectangle = cv.minAreaRect(cnt)
            box = cv.boxPoints(rectangle)
            box = np.int0(box)
            rectangle_area = cv.contourArea(box)
            rect_w, rect_h = rectangle[1][0], rectangle[1][1]
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)
            try:
                triangle = cv.minEnclosingTriangle(cnt)[1]
                triangle = np.int0(triangle)
                triangle_area = cv.contourArea(triangle)
            except:
                triangle_area = 0
            shapes_areas = {
                'circle': circle_area,
                'rectangle' if aspect_ratio > 1.5 else 'square': rectangle_area,
                'triangle': triangle_area,
            }
            diffs = {
                name: abs(area - shapes_areas[name]) for name in shapes_areas
            }
            shape_name = min(diffs, key=diffs.get)
            return shape_name, area


def calc_angle(color, imge):
    cnt = get_cont(imge, colors[color])
    if cnt:
        for contours in cnt:
            rect = cv.minAreaRect(contours)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            center = (int(rect[0][0]), int(rect[0][1]))
            area = int(rect[1][0] * rect[1][1])

            edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
            edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

            usedEdge = edge1
            if cv.norm(edge2) > cv.norm(edge1):
                usedEdge = edge2

            reference = (1, 0)  # horizontal edge
            angle_ = 180.0 / pi * acos(
                (reference[0] * usedEdge[0] + reference[1] * usedEdge[1]) / (cv.norm(reference) * cv.norm(usedEdge)))
            if area > 200:
                if angle_ is not None:
                    return angle_ if not isnan(angle_) else 0


def keep_yaw(yaw, power, P, D):
    to_180(yaw)
    try:
        error = auv.get_yaw() - yaw
        error = to_180(error)
        out = keep_yaw.reg.process(error)
        out = clamp(out, -100, 100)
        auv.set_motor_power(0, clamp((power - out), -100, 100))
        auv.set_motor_power(1, clamp((power + out), -100, 100))
    except:
        keep_yaw.reg = PD()
        keep_yaw.reg.set_p(P)
        keep_yaw.reg.set_d(D)


def keep_tube(angle_, power, P, D):
    try:
        out = clamp(angle_, -100, 100)
        auv.set_motor_power(0, clamp((power - out), -100, 100))
        auv.set_motor_power(1, clamp((power + out), -100, 100))
    except:
        keep_yaw.reg = PD()
        keep_yaw.reg.set_p(P)
        keep_yaw.reg.set_d(D)


def move_around(power, p):
    global angle
    _, imge = get_color('blue')
    try:
        angle = calc_angle('blue', imge)
        angle -= 90
    except:
        pass
    centralize_side('blue', 'yellow')
    keep_tube(angle * p, power, 1, 1)
    keep_depth(3, 20, 2.5)


print('--------Start--------')
depthing(3, 4)
for i in range(5):
    while True:
        move_around(25, 2)
        _, img = get_2color('yellow', 'green')
        try:
            shape, area = area_2color_shape('yellow', 'green', img)
            print(area)
            if area >= 700:
                break
        except:
            pass
    now_ang = auv.get_yaw()
    print('--------Stab--------')
    while not centralize('yellow', 'green', 3):
        centralize('yellow', 'green', 3)
    print('--------Stabbed--------')
    try:
        _, img = get_color('yellow')
        shape, area = area_shape('yellow', img)
        if area > 700:
            yellow += 1
    except:
        green += 1
    print('green:', green, '\n', 'yellow:', yellow)
    auv.drop()
    go(now_ang, 40, 3, 3, 'blue')
print('--------End--------')
s = green + yellow * 2
print(s)
while True:
    keep_depth(0, 20, 2.5)
    if s % 2 == 0:
        auv.set_motor_power(0, 100)
        auv.set_motor_power(1, -100)
    else:
        auv.set_motor_power(0, -100)
        auv.set_motor_power(1, 100)
# depthing(3, 7)
# while True:
#     move_around(25, 1.5)
