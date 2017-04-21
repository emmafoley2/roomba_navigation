#!/usr/bin/python

#### new camera code ####

#### building on original camera code but incorporating more accurate robot detection ####

#### robot will report back its position to the camera, camera will search only close to the reported position ####

##########################################################################
#   libraries
##########################################################################
import cv2
import numpy as np
from openni import *
from time import sleep
import threading
import matplotlib as mpl
from pylab import *
import rospy
from std_msgs.msg import String

##########################################################################
#   constants
##########################################################################
#camera/image properties
screenw=320
screenh=240
focaly=290
focalx=307
fps=30
minArea=30
maxArea=800     #area of contour to count as robot
camheight = 1400*1400 #camera height in mm, squared for use in calculations
#try to read camera height from text file
fin=open("grid.txt", "r")
line=fin.readline()
arr=line.strip().split(",")
camheight=float(arr[0])
print("camera height: ", camheight)
fin.close()
camheight*=camheight #square for use in calcs

lower_green = np.array([20,30,50])
upper_green = np.array([60,255,255]) #colour range for roomba tracking


##########################################################################
#   camera setup
##########################################################################

context = Context()
context.init()

depth = DepthGenerator()
depth.create(context)
depth.set_resolution_preset(RES_QVGA)
depth.fps = fps
context.start_generating_all()

image = ImageGenerator()
image.create(context)
image.set_resolution_preset(RES_QVGA)
image.fps = fps

user = UserGenerator()
user.create(context)

skel_cap = user.skeleton_cap
pose_cap = user.pose_detection_cap

##########################################################################
#   tracking callbacks
##########################################################################
def new_user(src,id):
    print "1/4 User {} detected. Looking for pose..." .format(id)
    skel_cap.request_calibration(id, True)

def pose_detected(src, pose, id):
    print "2/4 Detected pose {} on user {}. Requesting calibration..." .format(pose,id)
    pose_cap.stop_detection(id)
    skel_cap.request_calibration(id, True)

def calibration_start(src,id):
    print "3/4 Calibration started for user {}." .format(id)

def calibration_complete(src, id, status):
    if status == CALIBRATION_STATUS_OK:
        print "4/4 User {} calibrated successfully! Starting to track." .format(id)
        skel_cap.start_tracking(id)
    else:
        print "ERR User {} failed to calibrate. Restarting process." .format(id)
        new_user(user,id)
    
def lost_user(src, id):
    print "--- User {} lost." .format(id)

#Register the callbacks to the user node
user.register_user_cb(new_user, lost_user)
pose_cap.register_pose_detected_cb(pose_detected)
skel_cap.register_c_start_cb(calibration_start)
skel_cap.register_c_complete_cb(calibration_complete)

#set the profile
skel_cap.set_profile(SKEL_PROFILE_ALL)

#start generating
context.start_generating_all()

print "0/4 Starting to detect users. Press Ctrl-C to exit."


##########################################################################
#   functions
##########################################################################


def findRobot():
    global res
    global mask
    robot_detected=0
    prev_lowest_error=20000
    hsv = cv2.cvtColor(bgrframe, cv2.COLOR_BGR2HSV)     # convert color space to hsv for easier tracking
    mask = cv2.inRange(hsv, lower_green, upper_green)   # filter out sections color matching roomba
    res = cv2.bitwise_and(bgrframe, bgrframe, mask=mask)
    contours, hierarchy = cv2.findContours(mask, 1,2)  #calculate visible contours
    for i in range(len(contours)):
        M=cv2.moments(contours[i])
        if (cv2.contourArea(contours[i]) > minArea and cv2.contourArea(contours[i])<maxArea):    
            centre=np.array([int(M['m10']/M['m00']),int(M['m01']/M['m00'])])        #calculate centre of mass for contour
            c=centre
            dcentre=centre
            depth_robot=depthMap[dcentre[0],dcentre[1]] #find depth for centre pixel
            centre[0]-=(screenw/2)
            centre[1]-=(screenh/2)      #calc distance from centre of screen
            xrob=depth_robot*(centre[0])/focalx
            if (depth_robot>1400):
                yrob = np.sqrt(depth_robot*depth_robot - camheight)
            else:
                yrob=0
            pos=np.array([xrob, yrob])
            error=pos-robot_position
            error=np.linalg.norm(error)
            if(error<prev_lowest_error):
                cv2.circle(res, (dcentre[0],dcentre[1]),1,(255,255,255),5) #place a circle on res frame for debug
                cv2.circle(frame, (c[0],c[1]),1,(0,255,255),10)
                xrobot=xrob
                yrobot=yrob
                robot_detected = 1
                prev_lowest_error=error
    if(robot_detected==0):
        xrobot=-10
        yrobot=-10
        depth_robot=0
    
    return(xrobot,yrobot)
#possibly switch to returning coordinates instead of global variables
    
def findUser():
    #global user_pos
    for id in user.users:
        if skel_cap.is_tracking(id):
            h=skel_cap.get_joint_position(id, SKEL_TORSO)
            user_pos=[h.point[0],h.point[2]]
            return user_pos
    return [0,0]


##########################################################################
# ROS callbacks 
##########################################################################

def callback(data):
    global robot_position
    message=data.data.strip().split()
    message = map(float, message)
    #rospy.loginfo(message)
    robot_position=np.array([message[0], message[1]])
    robot_position=np.array([0,0])

#########################################################################

rospy.init_node('camera', anonymous=True)

robot_subscriber = rospy.Subscriber("robot_info", String, callback)
robot_publisher  = rospy.Publisher("cam_pos", String, queue_size=1)
user_publisher = rospy.Publisher("user_pos", String, queue_size=1)
#rospy.spinonce()

messagecount = 0
xrobot=0
yrobot=0
depth_robot=0
robot_detected=0
robotpoll=1
robot_position=np.array([0,0])
robot_grid=np.zeros((50,50))
user_pos="-1 -1" 
i=1

try:
    while not rospy.is_shutdown():
        user_pos="0 0"
        MESSAGE = np.array([0,0,0,0])
        context.wait_any_update_all()
        depthMap = depth.map
        frame = np.fromstring(depth.get_raw_depth_map_8(),"uint8").reshape(240,320)
        bgrframe = np.fromstring(image.get_raw_image_map_bgr(), dtype = np.uint8).reshape(image.metadata.res[1], image.metadata.res[0],3)
        
        robot_pos=findRobot()
        user_pos=findUser()

        robot_message=str(robot_pos[0])+" "+str(robot_pos[1])
        user_message=str(user_pos[0])+" "+str(user_pos[1])
        #message1=str(i)+" "+str(i)
        i+=1
        robot_publisher.publish(robot_message)
        #rospy.loginfo(message1)
        user_publisher.publish(user_message)
        sleep(0.15)
        rospy.loginfo(user_pos)
        #cv2.imshow('depth',frame)
        cv2.imshow('bgr',bgrframe)
        cv2.imshow('mask',mask)

        k=cv2.waitKey(5)&0xFF
        if k == 27:
            break
    
except KeyboardInterrupt or rospy.ROSInterruptException:
    print "Closing"
    cv2.destroyAllWindows()
    robotpoll=0
    rospy.signal_shutdown("end of program")
    print "all closed"
