# coding=utf8
import sys
import cv2
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *
import HiwonderSDK.Board as Board


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

class movy():
    def _init_(self, servo1):
        yes = None
        self.servo1 = servo1

    def movin(self, __isRunning, setBuzzer, world_X, world_Y, unreachable, rotation_angle, coordinate, detect_color):
        if __isRunning:        
            if detect_color != 'None' and start_pick_up:  #如果检测到方块没有移动一段时间后，开始夹取
                #移到目标位置，高度6cm, 通过返回的结果判断是否能到达指定位置
                #如果不给出运行时间参数，则自动计算，并通过结果返回
                self.set_rgb(detect_color)
                setBuzzer(0.1)
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) #如果可以到达指定位置，则获取运行时间

                    if not __isRunning:
                        exit
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle) #计算夹持器需要旋转的角度
                    Board.setBusServoPulse(1, self.servo1 - 280, 500)  # 爪子张开
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        exit
                    AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                    time.sleep(1.5)

                    if not __isRunning:
                        exit
                    Board.setBusServoPulse(1, self.servo1, 500)  #夹持器闭合
                    time.sleep(0.8)

                    if not __isRunning:
                        exit
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起
                    time.sleep(1)

                    if not __isRunning:
                        exit
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    
                    if not __isRunning:
                        exit                   
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        exit
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        exit                   
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        exit
                    Board.setBusServoPulse(1, self.servo1 - 200, 500)  # 爪子张开  ，放下物体
                    time.sleep(0.8)

                    if not __isRunning:
                        exit
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    self.initMove()  # 回到初始位置
                    time.sleep(1.5)

                    detect_color = 'None'
                    get_roi = False
                    start_pick_up = False
                    self.set_rgb(detect_color)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, self.servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)

        return detect_color, unreachable, get_roi, start_pick_up, servo2_angle
    

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
    
    def init(self):
        print("ColorSorting Init")
        self.initMove()

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

    def setBuzzer(timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)
    

