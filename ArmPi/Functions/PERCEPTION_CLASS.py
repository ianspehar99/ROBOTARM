# coding=utf8
import sys
import cv2
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK() #creates an instance of the armIK class

range_rgb = { #defines color ranges
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

class perception:
    def _init_(self):
        self.detect_color = 'None'
        self.rect = 'None'
        self.roi = ()
        pass
    def reset(self):
        #if we need to reset our variables
        self.detect_color = 'None'
        self.rect = 'None'
        self.roi = ()
        pass
    def getAreaMaxContour(self, contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #历遍所有轮廓, go through all the contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  #计算轮廓面积, calculate profile area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    #if the area is greater than 300
                    #the largest area of the profile is effective to filter out distractions
                    area_max_contour = c

        return area_max_contour, contour_area_max  #返回最大的轮廓, retrun the largest outline



    def camera(self,color_range, __target_color,frame_lab):
        
        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
    
        area_max = 0
        areaMaxContour = 0
        for i in color_range:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  #对原图像和掩模进行位运算
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))  #开运算
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8)) #闭运算
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
                areaMaxContour, area_max = self.getAreaMaxContour(contours)  #找出最大轮廓
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
    
        return  areaMaxContour_max,  color_area_max
    
    def obj_handling(self,areaMaxContour_max, size, img, last_x, last_y, color_area_max,start_pick_up, color_list):
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            
            roi = getROI(box) #获取roi区域
            get_roi = True
            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
             
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #转换为现实世界坐标
            
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color_area_max], 1) #绘制中心点
            
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) #对比上次坐标来判断是否移动
            last_x, last_y = world_x, world_y

            if not start_pick_up:
                if color_area_max == 'red':  #红色最大
                    color = 1
                elif color_area_max == 'green':  #绿色最大
                    color = 2
                elif color_area_max == 'blue':  #蓝色最大
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                # 累计判断
                if distance < 0.5:
                    count += 1
                    center_list.extend((world_x, world_y))
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1:
                        rotation_angle = rect[2] 
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        center_list = []
                        count = 0
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    center_list = []
                    count = 0

                if len(color_list) == 3:  #多次判断
                    # 取平均值
                    color = int(round(np.mean(np.array(color_list))))
                    color_list = []
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                    elif color == 3:
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                    else:
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
            return start_pick_up, get_roi, roi, distance, world_X, world_Y, detect_color, draw_color, rotation_angle, color_list
    