import cv2
import numpy as np
from ArmPi import getMaskROI, getAreaMaxContour
from ArmPi import getROI, getCenter, convertCoordinate
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
import Camera
from CameraCalibration.CalibrationConfig import *
import math
import time


#JUST DO ONE METHOD FOR NOW THIS IS RIDICULOUS


class PerceptionClass:
    def __init__(self, target_color ):
        # Initialize all variables as instance attributes
        self.roi = ()
        self.rect = None
        self.count = 0
        self.get_roi = False
        self.move_square = False
        self.center_list = []
        self.unreachable = False
        self.__isRunning = False
        self.start_pick_up = False
        self.last_x, self.last_y = 0, 0
        self.rotation_angle = 0
        self.world_X, self.world_Y = 0, 0
        self.start_count_t1 = True
        self.t1 = 0
        self.detect_color = 'None'
        self.draw_color = self.range_rgb["black"]
        self.color_list = []
        self.size = (640, 480)
        self.target_color = target_color
        self.range_rgb = {
            'red':   (0, 0, 255),
            'blue':  (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }


    def run(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        if self.get_roi and not self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)    

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0

        if not self.start_pick_up:
            for i in self.color_range:
                if i in self.__target_color:
                    frame_mask = cv2.inRange(frame_lab, self.color_range[i][0], color_range[i][1])  # 对原图像和掩模进行位运算
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
                    if areaMaxContour is not None:
                        if area_max > max_area:  # 找最大面积
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 2500:  # 有找到最大面积
                self.rect = cv2.minAreaRect(areaMaxContour_max)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = getROI(box)  # 获取roi区域
                self.get_roi = True

                img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)  # 获取木块中心坐标

                self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, size)  # 转换为现实世界坐标

                if not self.start_pick_up:
                    cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
                    cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', 
                                (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color_area_max], 1)  # 绘制中心点
                    distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))  # 对比上次坐标来判断是否移动
                self.last_x, self.last_y = self.world_x, self.world_y

                if not self.start_pick_up:
                    if color_area_max == 'red':  # 红色最大
                        color = 1
                    elif color_area_max == 'green':  # 绿色最大
                        color = 2
                    elif color_area_max == 'blue':  # 蓝色最大
                        color = 3
                    else:
                        color = 0
                    self.color_list.append(color)
                    # 累计判断
                    if distance < 0.5:
                        self.count += 1
                        self.center_list.extend((self.world_x, self.world_y))
                        if self.start_count_t1:
                            self.start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 0.5:
                            self.rotation_angle = self.rect[2]
                            self.start_count_t1 = True
                            self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                            self.center_list = []
                            self.count = 0
                            self.start_pick_up = True
                    else:
                        self.t1 = time.time()
                        self.start_count_t1 = True
                        self.center_list = []
                        self.count = 0

                if len(self.color_list) == 3:  # 多次判断
                    # 取平均值
                    color = int(round(np.mean(np.array(self.color_list))))
                    self.color_list = []
                    if color == 1:
                        self.detect_color = 'red'
                        self.draw_color = self.range_rgb["red"]
                    elif color == 2:
                        self.detect_color = 'green'
                        self.draw_color = self.range_rgb["green"]
                    elif color == 3:
                        self.detect_color = 'blue'
                        self.draw_color = self.range_rgb["blue"]
                    else:
                        self.detect_color = 'None'
                        self.draw_color = self.range_rgb["black"]
            else:
                if not self.start_pick_up:
                    self.draw_color = (0, 0, 0)
                    self.detect_color = "None"
        
        if self.move_square:
            cv2.putText(img, "Make sure no blocks in the stacking area", (15, int(img.shape[0]/2)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)    

        cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)

        return img

# # Main function to run the program
# if __name__ == '__main__':
#     color_palletizing = ColorPalletizing()  # Create an instance of the class
#     color_palletizing.init()
#     my_camera = Camera.Camera()
#     my_camera.camera_open()

#     while True:
#         img = my_camera.frame
#         if img is not None:
#             frame = img.copy()
#             Frame = color_palletizing.run(frame)  # Call the run method of the instance
#             cv2.imshow('Frame', Frame)
#             key = cv2.waitKey(1)
#             if key == 27:
#                 break

#     my_camera.camera_close()
#     cv2.destroyAllWindows()
