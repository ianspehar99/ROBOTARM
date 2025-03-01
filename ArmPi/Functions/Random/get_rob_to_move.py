#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
import math
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from ArmPi.Functions.Random.Perception_Functions import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

def init():
    print("ColorTracking Init")
    initMove()

def reset():
    global count, track, _stop, get_roi, first_move, center_list, __isRunning, detect_color, action_finish, start_pick_up, __target_color, start_count_t1   
    
    count = 0, _stop = False, track = False, get_roi = False, center_list = [], first_move = True, __target_color = (), detect_color = 'None', action_finish = True, start_pick_up = False, start_count_t1 = True   

# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

##VARIABLES (im in hell)
servo1 = 500
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
__target_color = ('red',)
get_roi = False
__isRunning = False
detect_color = 'None'
action_finish = True
start_pick_up = False
start_count_t1 = True
size = (640, 480)
roi = ()
last_x, last_y = 0, 0

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', )
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()

            #Use functions now


            frame_lab, get_roi = find_roi(frame, roi, get_roi, size, start_pick_up, __isRunning)
            areaMaxContour, max_area, detect_color = contour_detect(frame_lab, __target_color, color_range, start_pick_up)
            roi, get_roi, rect, world_x, world_y = box_detection(areaMaxContour, detect_color, frame, size, range_rgb)
            world_x, world_y, last_x, last_y, center_list, count, start_count_t1, t1, start_pick_up, detect_color, range_rgb = track_movement(world_x, world_y, rect, last_x, last_y, center_list, count, start_count_t1, t1, start_pick_up, detect_color, range_rgb)
            Frame = run(frame)         
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()






















