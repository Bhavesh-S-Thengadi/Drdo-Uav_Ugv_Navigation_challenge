#!/usr/bin/env python

import abc
from imp import cache_from_source
from threading import local
from typing import DefaultDict
import rospy
import message_filters # To Achieve Multiple subscriber
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 #3.2.0
import imutils
import mavros
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Twist, Point
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from prius_msgs.msg import Control

# import roadDetect_2
import roadDetect_4
# import roadDetect_5
# import roadDetect_6
from car_detection import car_detect_test as car_detect

mavros.set_namespace()

bridge = CvBridge()

lower = np.array([0,0,0])
upper = np.array([255,255,200])

X = 320
Y = 350

cx = 320
cy = 350

global prevx
global prevy
global flag
global croadmask
global prev_throttle

prev_throttle = 0.0
croadmask = np.zeros((480, 640))
prevx = 0
prevy = 0
flag = 1
big_sq = (0, 0)
small_sq = (0, 0)
big_sq_prev = (0, 0)
small_sq_prev = (0, 0)

global pt1, pt2, box
pt1 = np.array((320,200), dtype=np.int32)
pt2 = np.array((320,280), dtype=np.int32)
box = np.ones((4,2), dtype=np.int32)
box *= 320
blackout_flag = 0
dead_end_flag = 0
dead_end_flag_2 = 0

#drone velocity
twist = Twist()

twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0

uav_vel = rospy.Publisher(mavros.get_topic('setpoint_velocity','cmd_vel_unstamped'), Twist,queue_size=10)
car_vel = rospy.Publisher('/prius', Control,queue_size=10)

def posCb(msg):
        global local_pos, dead_end_flag, dead_end_flag_2 
        local_pos = Point()

        local_pos.x = msg.pose.position.x
        local_pos.y = msg.pose.position.y
        local_pos.z = msg.pose.position.z
        
        print("x: {}\ny: {}\nz: {}".format(local_pos.x, local_pos.y, local_pos.z))


        if (local_pos.x > 160) and (local_pos.x < 240) and (local_pos.y < -10) and (local_pos.y > -60):
            print("dead endddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd")
            dead_end_flag = 1
        elif (local_pos.x > 475) and (local_pos.x < 555) and (local_pos.y < 40) and (local_pos.y > -61):
            print("dead enddd2222222222222222222222222222222222222222222222222222222222")
            dead_end_flag = 2
        elif (local_pos.x > 555) and (local_pos.x < 645) and (local_pos.y < 138) and (local_pos.y > 40):
            print("dead enddd3333333333333333333333333333333333333333333333333333333333")
            dead_end_flag = 3
        elif (local_pos.x > -40) and (local_pos.x < 11) and (local_pos.y < 15) and (local_pos.y > -50):
            print("dead enddd4444444444444444444444444444444444444444444444444444444444")
            dead_end_flag = 4
        elif (local_pos.x > -205) and (local_pos.x < -40) and (local_pos.y < -50) and (local_pos.y > -144):
            print("dead enddd5555555555555555555555555555555555555555555555555555555555")
            dead_end_flag = 5
        else:
            dead_end_flag = 0

# Node is subscribing to the /mavros/local_position/pose
pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, posCb)

class Kinect_Node(object):
    def __init__(self):
        #use CvBridge to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Node is subscribing to the rgb/image_raw topic
        self.rgb_sub = message_filters.Subscriber('/depth_camera/rgb/image_raw', Image)

        # Node is subscribing to the /depth_to_rgb/image_raw topic
        self.depth_sub = message_filters.Subscriber('/depth_camera/depth/image_raw', Image)
    
    def callback(self, ros_rgb, ros_depth):
        global rgb_frame,depth_frame
        depth_frame = self.br.imgmsg_to_cv2(ros_depth, desired_encoding='32FC1')
        rgb_frame = self.br.imgmsg_to_cv2(ros_rgb, "bgr8")
        car_controller()
        drone_controller()
        # show()

def setMavFrame():
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', mavros_msgs.srv.SetMavFrame)
            flightModeService(mav_frame=8)
        except rospy.ServiceException as e:
            print("ROSservice call failed: {}".format(e))

def show():
    global rgb_frame,depth_frame
    cv2.imshow("depth camera", depth_frame)
    cv2.imshow("rgb camera", rgb_frame)
    cv2.waitKey(1)

def drone_controller():
    global carForward, center, blackout_flag, flag
    follow_car(carForward, center, blackout_flag, flag)

    # cv2.arrowedLine(croadmask, pt1, pt2, (0,0,0), 4)
    # cv2.circle(croadmask, center, 10, (0, 255, 0), -1)
    # follow_car(car_vector, center)
    # cv2.imshow("mask", croadmask)
    # cv2.waitKey(1)
    # global big_sq_prev, small_sq_prev, big_sq, small_sq
    # global rgb_frame, flag
    
    # #img = bridge.imgmsg_to_cv2(data,desired_encoding=data.encoding)
    # rgb_frame = imutils.resize(rgb_frame, width=600)
    # img = rgb_frame
    # img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    
    
    # img_hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    # mask = cv2.inRange(img_hsv,lower,upper)
    # # cv2.imshow("mask",mask)

    # edged = cv2.Canny(mask, 170, 255) #Determine edges of objects in an image
    # ret,thresh = cv2.threshold(edged,240,255,cv2.THRESH_BINARY)  
    # contours = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    # areas = np.array([])
    # centers = []
    # contours = contours[0] if len(contours) == 2 else contours[1]
    # for cntr in contours:
    #     x,y,w,h = cv2.boundingRect(cntr)
    #     ct = (x+(w/2), y+(h/2))
    #     areas = np.append(areas, w*h)
    #     centers.append(ct)
    #     cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)

    # try:
    #     big_sq = centers[np.argmax(areas)]
    #     small_sq = centers[np.argmin(areas)]
    #     big_sq = (int(big_sq[0]), int(big_sq[1]))
    #     small_sq = (int(small_sq[0]), int(small_sq[1]))
    #     cv2.arrowedLine(img, big_sq, small_sq, (0, 0, 255), 3)
    #     follow_car(big_sq, small_sq)
    #     big_sq_prev = big_sq
    #     small_sq_prev = small_sq
    #     flag = 1 # car in view
    #     print("Car in View: Flag = {}".format(flag))
    # except ValueError:
    #     big_sq = big_sq_prev
    #     small_sq = small_sq_prev
    #     follow_car(big_sq, small_sq)
    #     print("ValueError")
    #     flag = 0 # car lost
    #     print("Car Lost: Flag = {}".format(flag))
    # except NameError:
    #     print("NameError")

    # cv2.circle(img, big_sq, 7, (0, 0, 255), -1)
    # cv2.circle(img, (X, Y), 7, (255, 255, 0), -1)
    # cv2.arrowedLine(img, (X,Y), big_sq, (0, 255, 0), 3)
    # cv2.arrowedLine(img, (320,280), (320,0), (0, 0, 0), 3)
    
    # cv2.imshow("img",img)
    # cv2.waitKey(1)

def follow_car(car_vector, center, blackout_flag, flag):
    global prevx, prevy, carForward
    cx = center[0]
    cy = center[1]
    center_precision_x = (cx - X)
    center_precision_y = (cy - Y)
    error_margin = 0
    yaw_margin = 5   
    
    vx = center_precision_x
    vy = center_precision_y
    a = vx**2+vy**2
    v = math.sqrt(a)
    
    v1 = car_vector
    v2 = (0, -280)
    angle = math.acos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))*(180/np.pi)
    # print("v1 = {}, angle = {}".format(v1, angle))

    wx,wy=0,0
    if (not blackout_flag) or (flag):
        ## following car
        if(v >= error_margin):
            vel = 1
            
            propx = -(center_precision_y)
            derivx = (center_precision_y - prevy)

            propy = -(center_precision_x)
            derivy = (center_precision_x - prevx)


            P = 0.004
            D = 0.0001 


            wy = vel*(P*propy + D*derivy)
            wx = vel*(P*propx + D*derivx)
        else:
            wx = 0
            wy = 0

        prevx = center_precision_x
        prevy = center_precision_y
        
        ##Height adjustment
        height = depth_frame[240][320]
        print("Height: {}\n".format(height))
        height_param = 18
        if height > height_param:
            wz = -0.3
        elif height < (height_param - 1):
            wz = 0.3
        else:
            wz = 0.0

        ## adjusting yaw
        if v1[0] < 0:
            #anticlockwise
            if (angle >= yaw_margin):
                yaw = 0.4
            else:
                yaw = 0.0
        else:
            #clockwise
            if (angle >= yaw_margin):
                yaw = -0.4
            else:
                yaw = 0.0
    else:
        # print("Dead End. Drone going forward ddddddddddddddddddddddddddddddd")
        wx = 0.03
        wy = 0.0
        wz = 0.0
        yaw = 0.0
        print("wx: {}, wy: {}, wz: {}, yaw: {}".format(wx, wy, wz, yaw))
    
    twist.linear.x = wx
    twist.linear.y = wy
    twist.linear.z = wz
    twist.angular.z = yaw
    uav_vel.publish(twist)

def follow_road(angle, flag, blackout_flag):
    # global depth255, prev_throttle
    # a = np.mean(depth255[110:130, 300:340])
    # # print(depth255[110:130, 300:340].shape)
    # b = np.mean(depth255[350:370, 300:340])
    # # print(depth255[350:370, 300:340].shape)
    # slope = (a - b)
    # print("slope: {}".format(slope))

    # throttle = 150/abs(slope)
    # print(throttle)

    throttle = 0.2
    # throttle = (prev_throttle + throttle)/2
    # prev_throttle = throttle

    brake = 0.0
    shift_gears = 2
    TOL = 5
    if not blackout_flag:
        if flag:
            if abs(angle) > TOL:
                if angle > 0:
                    steer = -0.45
                else: # angle < 0
                    steer = 0.45
            else:
                steer = 0
        else:
            brake = 5.0
            steer = 0.01
            throttle = 0.0
            shift_gears = 1
    else:
        print("Image Blacked OUTwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww")
        throttle = 0.005
        brake = 1
        steer = 0.0
        shift_gears = 1

    
    control = Control()
    control.throttle = throttle
    control.brake = brake
    control.steer = steer
    control.shift_gears = shift_gears
    car_vel.publish(control)
    print('steer:{}, throttle:{}'.format(steer, throttle))    

def car_controller():
    global targetVector, depth255, depth_frame, pt1, pt2, box, flag, croadmask, carForward, center, blackout_flag, dead_end_flag, dead_end_flag_2
    proc =  roadDetect_4.RoadImageProcessor(pt1, pt2, box)

    current_frame = depth_frame
    current_frame = np.nan_to_num(current_frame, nan=35, posinf=20, neginf=20)

    depths = current_frame
    depth255 = cv2.normalize(current_frame, None, 255, 0, cv2.NORM_MINMAX,cv2.CV_8U)
    # cv2.imshow("depths", depth255)
    # cv2.waitKey(1)

    depth3Channel = cv2.cvtColor(depth255, cv2.COLOR_GRAY2BGR)

    proc.loadImages(depth3Channel, depths)
    croadmask, targetAngle, pt1, pt2, box, carForward, center, flag, blackout_flag, dead_end_flag, dead_end_flag_2 = proc.targetVector(blackout_flag, dead_end_flag, dead_end_flag_2)

    follow_road(targetAngle, flag, blackout_flag)

    cv2.imshow("camera", croadmask)
    cv2.waitKey(1)
            
def main():
    my_node = Kinect_Node()
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name. 
    rospy.init_node("kinect_sub_py", anonymous=True)

    ts = message_filters.ApproximateTimeSynchronizer([my_node.rgb_sub, my_node.depth_sub], 10, 0.1)
    ts.registerCallback(my_node.callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    setMavFrame()
    main()