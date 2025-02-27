import cv2
import numpy as np
import math
import time
from ArmPi import getMaskROI, getAreaMaxContour, getROI, getCenter, convertCoordinate


def find_roi(img, roi, get_roi, size, start_pick_up, __isRunning):
    if not __isRunning:
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)    
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    return frame_lab, get_roi

def contour_detect(frame_lab, target_color, color_range, start_pickup):
    max_area = 0
    areaMaxContour = None
    detect_color = None
    if not start_pickup:
        for color in target_color:
            frame_mask = cv2.inRange(frame_lab, color_range[color][0], color_range[color][1])
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
            
            if areaMaxContour is not None and area_max > max_area:
                max_area = area_max
                detect_color = color

        return areaMaxContour, max_area, detect_color

def box_detection(areaMaxContour, detect_color, img, size, range_rgb):
    rect = cv2.minAreaRect(areaMaxContour)
    box = np.int0(cv2.boxPoints(rect))
    
    roi = getROI(box)
    get_roi = True
    square_length = max(rect[1])
    img_centerx, img_centery = getCenter(rect, roi, size, square_length)
    world_x, world_y = convertCoordinate(img_centerx, img_centery, size)
    
    cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
    cv2.putText(img, f'({world_x},{world_y})', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)
    
    return roi, get_roi, rect, world_x, world_y

def track_movement(world_x, world_y, rect, last_x, last_y, center_list, count, start_count_t1, t1, start_pick_up, detect_color, range_rgb):
    distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))
    last_x, last_y = world_x, world_y
    
    if distance < 0.3:
        center_list.extend((world_x, world_y))
        count += 1
        if start_count_t1:
            start_count_t1 = False
            t1 = time.time()
        if time.time() - t1 > 1.5:
            rotation_angle = rect[2]  #WOULD WANT TO RETURN THESE WHEN DOING MOVEMENT SHIT
            start_count_t1 = True
            world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
            count = 0
            center_list = []
            start_pick_up = True
    else:
        t1 = time.time()
        start_count_t1 = True
        count = 0
        center_list = []
    
    return world_x, world_y, last_x, last_y, start_pick_up, start_count_t1, t1, count, center_list

