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

from perception_class_simple import PerceptionClass

class move(PerceptionClass):
    def __init__(self):
        super().__init__()
        

    def Move(self):
        
        # 不同颜色木快放置坐标(x, y, z)
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        while True:
            if self.__isRunning:
                if first_move and start_pick_up: # 当首次检测到物体时               
                    #action_finish = False
                    self.set_rgb(detect_color)
                    self.setBuzzer(0.1)               
                    result = AK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0) # 不填运行时间参数，自适应运行时间
                    if result == False:
                        unreachable = True
                    else:
                        unreachable = False
                    time.sleep(result[2]/1000) # 返回参数的第三项为时间
                    start_pick_up = False
                    first_move = False
                    #action_finish = True
                elif not first_move and not unreachable: # 不是第一次检测到物体
                    self.set_rgb(detect_color)
                    if track: # 如果是跟踪阶段
                        if not self.__isRunning: # 停止以及退出标志位检测
                            continue
                        AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)                    
                        track = False
                    if start_pick_up: #如果物体没有移动一段时间，开始夹取
                        #action_finish = False
                        if not self.__isRunning: # 停止以及退出标志位检测
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 280, 500)  # 爪子张开
                        # 计算夹持器需要旋转的角度
                        servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                        time.sleep(2)
                        
                        if not  self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1, 500)  # 夹持器闭合
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        # 对不同颜色方块进行分类放置
                        result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not self.__isRunning:
                            continue
                        servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 200, 500)  # 爪子张开，放下物体
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue                    
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.initMove()  # 回到初始位置
                        time.sleep(1.5)

                        detect_color = 'None'
                        first_move = True
                        self.get_roi = False
                        #action_finish = True
                        start_pick_up = False
                        self.set_rgb(detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if _stop:
                    _stop = False
                    Board.setBusServoPulse(1, self.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

    def setBuzzer(self,timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def set_rgb(self,color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()