#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from PERCEPTION_CLASS import perception
from MOVING_CLASS import movy

p = perception()
m = movy()


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red')
# 设置检测颜色
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# 夹持器夹取时闭合的角度
servo1 = 500

#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True
def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True


def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorSorting Stop")

def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorSorting Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False 
world_X, world_Y = 0, 0
# def move():
#     global rect
#     global _stop
#     global get_roi
#     global unreachable
#     global __isRunning
#     global detect_color
#     global start_pick_up
#     global rotation_angle
#     global world_X, world_Y
#     global servo1
#     global servo2_angle
    
    
#     #放置坐标
#     coordinate = {
#         'red':   (-15 + 0.5, 12 - 0.5, 1.5),
#         'green': (-15 + 0.5, 6 - 0.5,  1.5),
#         'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
#     }
#     while True:
#         detect_color, unreachable, get_roi, start_pick_up, 
#         servo2_angle  = m.movin( __isRunning, world_X, world_Y,
#         unreachable, rotation_angle, coordinate)
        
        
#运行子线程
# th = threading.Thread(target=move)
# th.setDaemon(True)
# th.start()    

t1 = 0
roi = ()
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]
def run(img):
    global roi
    global rect
    global count
    global get_roi
    global center_list
    global unreachable
    global __isRunning
    global start_pick_up
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global start_count_t1, t1
    global detect_color, draw_color, color_list
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
    if get_roi and not start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    color_area_max = None
    max_area = 0
    
    
    if not start_pick_up:
        areaMaxContour_max,  color_area_max = p.camera(color_range, __target_color,frame_lab)
        if max_area > 2500:  # 有找到最大面积
            start_pick_up, get_roi, 
            roi,world_X, world_Y, detect_color, 
            draw_color, rotation_angle, 
            color_list = p.obj_handling(areaMaxContour_max, size, img, last_x, last_y, color_area_max,start_pick_up, color_list)
        else:
            if not start_pick_up:
                draw_color = (0, 0, 0)
                detect_color = "None"

    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    return img

if __name__ == '__main__':
    p.init()
    start()
    __target_color = ('red', 'green', 'blue')
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
