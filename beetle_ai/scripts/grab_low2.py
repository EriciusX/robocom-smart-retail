#encoding: UTF-8
#!/usr/bin/env python2
from tkinter import W
import cv2
import os
import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
from pymycobot.mycobot import MyCobot
from opencv_yolo import yolo
from VideoCapture import FastVideoCapture
import math
from GrabParams import grabParams
import basic
import argparse

parser = argparse.ArgumentParser(description='manual to this script')
parser.add_argument("--debug", type=bool, default="True")
args = parser.parse_args()

height_bias = grabParams.height_bias
done = grabParams.done

class Detect_marker(object):
    def __init__(self):
        super(Detect_marker, self).__init__()
        rospy.init_node('grab_low', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10) # 10hz
        self.mc = MyCobot(grabParams.usb_dev, grabParams.baudrate)
        self.mc.power_on() 
        self.yolo = yolo()
        self.c_x, self.c_y = grabParams.IMG_SIZE/2, grabParams.IMG_SIZE/2
        self.ratio = grabParams.ratio
        self.lv = 940 # cm
        self.hr_acuro = 2.1 #cm
        self.miss_count = 0
        self.class_list = []
        self.direction = 0  #left:0,right:1
        self.aruco_count = 0

    def move(self, x, y, dist):
        global height_bias, done
        if self.direction:
            coords = grabParams.coords_low_left
            coords_target = [coords[0],  coords[1]+int(y/3),  grabParams.grab_low_left, coords[3] + grabParams.pitch_low_left,  coords[4] + grabParams.roll_low_left,  coords[5]]
        else:
            coords = grabParams.coords_low_right
            coords_target = [coords[0],  coords[1]+int(y),   grabParams.grab_low_right, coords[3] + grabParams.pitch_low_right,  coords[4] + grabParams.roll_low_right,  coords[5] - int(y/2)]
        self.mc.send_coords(coords_target, 80, 0)
        time.sleep(0.2)
        self.mc.set_color(255,0,0)  #抓取，亮红灯
        self.go(dist) 
        time.sleep(0.2)
        basic.grap(True)
        time.sleep(0.2)
        self.back() 
        time.sleep(0.5)
        self.set_down()  
        done = True
        self.mc.set_color(0,255,0) #抓取结束，亮绿灯

    def get_position(self, x, y):
        wx = wy = 0
        if grabParams.grab_direct == "right":
            wx = (self.c_x - x) * self.ratio
            wy = (y - self.c_y) * self.ratio
        return wx, wy
            
    def transform_frame(self, frame):
        frame, ratio, (dw, dh) = self.yolo.letterbox(frame, (grabParams.IMG_SIZE, grabParams.IMG_SIZE))
        return frame
   
    def obj_detect(self, img):
        global done
        x=y=0
        w=h=0
        net = cv2.dnn.readNetFromONNX(grabParams.ONNX_MODEL)
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (grabParams.IMG_SIZE, grabParams.IMG_SIZE), [0, 0, 0], swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(net.getUnconnectedOutLayersNames())[0]
        boxes, classes, scores = self.yolo.yolov5_post_process_simple(outputs)
        if boxes is not None:
            for i in range(len(classes)):
                if classes[i] == 4:
                    self.class_list.append(i)
            if len(self.class_list):
                scores_max = scores[self.class_list[0]]
                right_target = self.class_list[0]
                for i in range(len(self.class_list)):
                    if scores[self.class_list[i]] > scores_max:
                        scores_max = scores[self.class_list[i]]
                        right_target = self.class_list[i]
                self.yolo.draw(img, zip(boxes)[right_target], zip(scores)[right_target], zip(classes)[right_target])
                left, top, right, bottom = boxes[right_target]
                x = int((left+right)/2)
                y = int((top+bottom)/2)
                w = bottom - top
                h = right - left    
            else:
                done = True
                self.mc.set_color(255,192,203) #识别不到，亮粉灯
                return None
        else:
            self.miss_count+=1
            if self.miss_count == 5:
                done = True
                self.mc.set_color(255,192,203) #识别不到，亮粉灯
            return None
        if w > h:
            width = h
        else: 
            width = h
        if x+y > 0:
            return x, y, width
        else:
            return None
    
    def distance(self, w):
        dist = self.hr_acuro / w * self.lv
        dist = dist - 9 - grabParams.go_diff
        return dist

    def aruco(self, frame):
        global done
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        while(1):
            self.aruco_count += 1
            corners, _, _ = cv2.aruco.detectMarkers(
                gray, cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250), parameters=cv2.aruco.DetectorParameters_create()
                )
            if len(corners) > 0:
                x = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                x = x/4.0
                x_size_p = abs(x - corners[0][0][0][0])*2
                dist = self.distance(x_size_p)
                return dist
            elif self.aruco_count == 3:
                done = True
                self.mc.set_color(255,192,203) #识别不到，亮粉灯
                return None

    def run(self):
        self.mc.set_color(0,0,255)#blue, arm is busy
        f = open("/home/robuster/beetle_ai/scripts/direction.txt", "r+")
        self.direction = int(f.read())
        f.write('1')
        f.close()

    def go(self, dist):
        if self.direction:
            count_max = int(dist + grabParams.move_power_low_left + 0.5)
        else:
            count_max = int(dist + grabParams.move_power_low_right + 0.5)
        count = 0
        print(dist, count_max)
        move_cmd = Twist()
        print(dist, count_max)
        time.sleep(0.5)
        while(1):
            self.pub.publish(move_cmd)
            count += 1
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0
            if count_max - count < 2:
                move_cmd.linear.x = 0.05 
                move_cmd.angular.z = 0
            if count == count_max:
                break
            self.rate.sleep()

    def back(self):
        count = 6
        move_cmd = Twist()
        move_cmd.linear.x = -0.2
        if grabParams.set_down_direction == "right":
            move_cmd.angular.z = -0.3
        else:
            move_cmd.angular.z = 0.3
        time.sleep(0.5)
        while(count):
            self.pub.publish(move_cmd)
            count-=1
            self.rate.sleep()

    def set_down(self):
        if grabParams.set_down_direction == "right":
            angle = self.mc.get_angles()
            self.mc.send_angles([angle[0] - 90, angle[1], angle[2], angle[3] + 10 ,angle[4], angle[5]], 70)
        else:
            angle = self.mc.get_angles()
            self.mc.send_angles([angle[0] + 90, angle[1], angle[2], angle[3] + 10 ,angle[4], angle[5]], 70)
        time.sleep(4)
        basic.grap(False)

    def show_image(self, img):
        if grabParams.debug and args.debug:
            cv2.imshow("figure", img)
            cv2.waitKey(500) 
        
def main():
    detect = Detect_marker()
    detect.run()
    cap = FastVideoCapture(grabParams.cap_num)
    time.sleep(1) 

    while cv2.waitKey(1) < 0 and not done:
        frame = cap.read()
        frame = detect.transform_frame(frame)
        detect.show_image(frame)
        detect_result = detect.obj_detect(frame)
        detect.show_image(frame)
        if detect_result is None:           
            continue
        else:            
            dist_aruco = detect.aruco(frame)
            x, y, w = detect_result
            real_x, real_y = detect.get_position(x, y)
            detect.move(real_x + grabParams.x_bias, real_y + grabParams.y_bias, dist_aruco)
            cap.close()
            
if __name__ == "__main__":
    main()
    