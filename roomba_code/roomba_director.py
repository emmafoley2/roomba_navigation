#!/usr/bin/python
from pyroomba import *
from time import sleep
from serial import *
from random import randint
from std_msgs.msg import String

from myroomba import *

import rospy
import sys
import math
import time
import datetime
import numpy as np
import threading

cam_update=1
drivespeed=400
tolerance=250

def route_callback(data):
    global route
    r=[]
    #rospy.loginfo(data.data)
    message=data.data.strip().split()
    message=map(float, message)
    i=0
    while i < len(message):
        r.append(np.array([message[i], message[i+1]]))
        i+=2
    if len(r) > 0:
        route=r


def cam_callback(data):
    global cam_pos
    global robot
    #rospy.loginfo(data.data)
    message=data.data.strip().split()
    message=map(float, message)
    cam_pos=np.array(message)
    error=pos-cam_pos
    updated=" no"
    if(np.linalg.norm(error) < 500 and np.linalg.norm(pos) > 1000 and cam_update==1):
        str1 = "Updated "+ str(pos) +" to " + str(cam_pos)
        updated=" updated"
       # print str1
        robot.position=cam_pos#(robotpos+campos)/2
    logfile.write(str(cam_pos)+","+str(pos)+","+str(error)+","+str(col)+updated+"\n")
    logfile.flush()



class send_thread(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID=threadID
        self.name = name
        self.counter = counter
    def run(self):
        global pos
        global col
        global route_flag
        print "Starting "+self.name
        while update:
            pos=robot.get_position()
            col=robot.get_collisions()
            message=" ".join([str(p) for p in pos])+" "+str(col)+" "+str(route_flag)
            publisher.publish(message)
            #rospy.loginfo(message)
        print "Exiting "+self.name

def shutdown():
    robot.stop()
    robot.finish()
    update=0
    send_thread1.join()
    rospy.signal_shutdown("goodreason")        

if __name__=='__main__':
    
    route=[0,0]
    cam_pos=np.array([10,10])
    pos=np.array([1, 1])
    col=0
    route_flag=2

    filename="log_t4"+str(cam_update)+time.strftime("%Y%m%d-%H%M%S")+".txt"
    logfile=open(filename, "w")
    logfile.write("Camera observation, robot report, error\n")
    logfile.flush()
   
    rospy.init_node('robot', anonymous=True)

    route_subscriber = rospy.Subscriber("route", String, route_callback)    
    cam_subscriber = rospy.Subscriber("cam_pos", String, cam_callback)
    publisher = rospy.Publisher("robot_info", String, queue_size=10)


    robot=myroomba("/dev/ttyMFD1", drivespeed)
    robot.startfull()
    coll_max=5


    update=1
    threads=[]
    send_thread1=send_thread(1, "Position publisher thread", 1)
    threads.append(send_thread1)
    send_thread1.start()
    
    sleep(2)

    while 1:
        
        while 1:
            destination = route[-1]
            print "Destination: ",destination
            print "Route", route
            go=raw_input("go? (y/n)")
            if go=='y':
                break
            if go=='q':
                shutdown()
        robot.back(500)

        route_flag = 0 
        
        colls=0
        goal_reached=0
        destination = route[-1]
        print "Destination: ", destination
        while colls<coll_max and not goal_reached: #while destination or maximum collisions not reached
            for r in route: #for each point in the current route
                if(robot.goto(r,tolerance))==2 and np.linalg.norm(destination-pos) >= tolerance: #collision, take new route
                    colls+=1 #increase collision count
                    sleep(1) #wait to ensure new route is ready
                    break   #start new route
            if np.linalg.norm(destination-pos) < tolerance:
                print "Reached goal"
                goal_reached=1

        route_flag=1

        while 1:
            destination = route[-1]
            print "Destination: ", destination
            print "route", route
            go=raw_input("return home? (y/n)")
            if go == 'y':
                break
            if go == 'q':
                shutdown()
    
        
        colls=0
        goal_reached=0
        while colls<coll_max and not goal_reached: #while destination or maximum collisions not reached
            for r in route: #for each point in the current route
                if(robot.goto(r,tolerance))==2: #collision, take new route
                    colls+=1 #increase collision count
                    sleep(1) #wait to ensure new route is ready
                    break   #start new route
            #if np.linalg.norm(destination-pos) < 500:
                print "Reached goal"
                goal_reached=1
               # break
        
        route_flag=2
  
        robot.dock()
        route_flag=2

    robot.stop()
    robot.finish()
    update=0
    send_thread1.join()
    rospy.signal_shutdown("goodreason")        
