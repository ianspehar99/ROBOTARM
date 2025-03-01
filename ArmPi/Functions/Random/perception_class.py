
import cv2
import numpy as np
from ArmPi import getMaskROI, getAreaMaxContour
from ArmPi import getROI, getCenter, convertCoordinate
import math
import time

class Perception:
    def __init__(self,img, roi, rect, count, track, get_roi, center_list, __isRunning, unreachable, 
                 detect_color, action_finish, rotation_angle, last_x, last_y, 
                 world_X, world_Y, world_x, world_y, start_count_t1, t1, 
                 start_pick_up, first_move, size, _target_color, range_rgb):
        self.img = img
        self.roi = roi
        self.rect = rect
        self.count = count
        self.track = track
        self.get_roi = get_roi
        self.center_list = center_list
        self.__isRunning = __isRunning
        self.unreachable = unreachable
        self.detect_color = detect_color
        self.action_finish = action_finish
        self.rotation_angle = rotation_angle
        self.last_x = last_x
        self.last_y = last_y
        self.world_X = world_X
        self.world_Y = world_Y
        self.world_x = world_x
        self.world_y = world_y
        self.start_count_t1 = start_count_t1
        self.t1 = t1
        self.start_pick_up = start_pick_up
        self.first_move = first_move
        self.size = size
        self._target_color = _target_color
        self.range_rgb = range_rgb
        self.color_area_max = None
        self.max_area = 0
        self.areaMaxContour_max = 0
        self.draw_color = None
        self.color_list = []
        self.detect_color = None
        self.color_range = {
            'red':   ((0, 0, 100), (100, 100, 255)),
            'green': ((35, 43, 46), (99, 255, 255)),
            'blue':  ((100, 43, 46), (124, 255, 255)),
        }
        
    def find_roi(self):
        # Process the image and isolate the ROI if we found contours in previous loop
        # Creates crosshairs, focuses in on roi if its ready
        # TODO:Need to find the dependencies for isrunning and getmaskroi
        img = self.img
        roi = self.roi
        size = self.size
        start_pick_up = self.start_pick_up
        __isRunning = self.__isRunning
        get_roi = self.get_roi
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        if not __isRunning:
            return img
        
        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        #如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
        if get_roi and start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, roi, size)    
        
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        self.get_roi = get_roi
        
        return frame_lab
    
    def contour_detect(self,frame_lab):
        # Detects contours in the ROI
        __target_color = self._target_color
        color_range = self.color_range
        detect_color = self.detect_color
        for i in color_range:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # 对原图像和掩模进行位运算
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积
                        max_area = area_max
                        self.color_area_max = i
                        self.areaMaxContour_max = areaMaxContour

        self.detect_color = detect_color
        return areaMaxContour, area_max, detect_color
    
    def box_detection(self,areaMaxContour, detect_color):
        # Detects the box and calculates the center of the box, and movement and shit
        size = self.size
        range_rgb = self.range_rgb
        img = self.img
        
        #TODO: Figure out last_x

        rect = cv2.minAreaRect(areaMaxContour)
        box = np.int0(cv2.boxPoints(rect))

        roi = getROI(box) #获取roi区域
        get_roi = True

        square_length = max(rect[1])
        img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
        world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标
        
        
        
        cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #绘制中心点
        distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
        last_x, last_y = self.world_x, self.world_y
        track = True

        self.roi = roi
        self.get_roi = get_roi
        self.rect = rect
        self.world_x = world_x
        self.world_y = world_y
        self.last_x = last_x
        self.last_y = last_y
        self.track = track
    
        return distance 
    
    def track_movement(self,distance):
        world_x = self.world_x
        world_y = self.world_y
        rect = self.rect
        t1 = self.t1
        start_count_t1 = self.start_count_t1
        center_list = self.center_list
        count = self.count
        color_area_max = self.color_area_max
        color_list = self.color_list
        start_pick_up = self.start_pick_up
        detect_color = self.detect_color
        draw_color = self.draw_color

        if color_area_max == 'red':  #红色最大
                    color = 1
        elif color_area_max == 'green':  #绿色最大
            color = 2
        elif color_area_max == 'blue':  #蓝色最大
            color = 3
        else:
            color = 0
        self.color_list.append(color)

        if distance < 0.3:
            center_list.extend((world_x, world_y))
            count += 1
            if start_count_t1:
                start_count_t1 = False
                t1 = time.time()
            if time.time() - t1 > 1.5:
                self.rotation_angle = rect[2]
                start_count_t1 = True
                self.world_X, self.world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                count = 0
                center_list = []
                self.start_pick_up = True
        else:
            t1 = time.time()
            start_count_t1 = True
            count = 0
            center_list = []
        
        if len(self.color_list) == 3:  #多次判断
            # 取平均值
            color = int(round(np.mean(np.array(color_list))))
            color_list = []
            if color == 1:
                detect_color = 'red'
                draw_color = self.range_rgb["red"]
            elif color == 2:
                detect_color = 'green'
                draw_color = self.range_rgb["green"]
            elif color == 3:
                detect_color = 'blue'
                draw_color = self.range_rgb["blue"]
            else:
                detect_color = 'None'
                draw_color = self.range_rgb["black"]

            self.start_count_t1 = start_count_t1
            self.count = count
            self.center_list = center_list
            self.t1 = t1
            self.start_pick_up = start_pick_up
            self.detect_color = detect_color
            self.draw_color = draw_color
            self.color_list = color_list

    
            
            

