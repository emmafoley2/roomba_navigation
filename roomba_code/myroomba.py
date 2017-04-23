from pyroomba import *
from time import sleep
from serial import *
from random import randint

import time
import mraa
import datetime
import socket
import select
import numpy as np
import math

led=mraa.Gpio(13)
led.dir(mraa.DIR_OUT)

class myroomba:
    
    def __init__(self, portname, drivespeed):
        self.roomba=Roomba(portname)
        self.drivespeed=drivespeed
        self.position = np.array([0.0,0.0])
        self.orientation=np.pi*1.5
        self.collisions=0
        self.ltbumper=0
        self.crashbumper=0
        #filename="log_"+time.strftime("%Y%m%d-%H%M%S")+".txt"
        #self.logfile=open(filename,"w")
        self.ticks_rotation=508.8
        self.wheeldiam=72
        self.wheelbase=235
        self.encoder_max=65536
        self.updateflag=0
        self.initial_sensors()
    
    def drive(self, mm=-1):
        self.update_position()
        collisions=0
        if mm < 0:
            self.roomba.drive(self.drivespeed, 0)
        else:
            distance=0
            start_pos=self.get_position().copy()
            self.roomba.drive(self.drivespeed,0)
            while(abs(distance) < abs(mm) and (collisions == 0)):
                self.update_position()
                distance=np.linalg.norm(self.position - start_pos)
                collisions=self.collisions
                #print ("drove", distance," of ", mm) 
            self.roomba.drive(0,0)
            return(collisions)
        

    def turn(self, rads=-11000):
        self.update_position()
        if rads<-10000:
            self.roomba.drive_direct(self.drivespeed,-self.drivespeed)
        else:
            angle=rads
            goal_angle=(self.get_orientation()+rads)%(2*np.pi)
            #start_angle=self.get_orientation()
            #print "Orientation: ", start_angle
            if rads<0:
                self.roomba.drive_direct(-self.drivespeed, self.drivespeed)
            else:
                self.roomba.drive_direct(self.drivespeed, -self.drivespeed)
            while(abs(angle) > np.pi/18):
                self.update_position()
                new_angle=self.get_orientation() 
                angle = goal_angle-new_angle
            self.roomba.drive(0,0)

    def back(self, mm=-1):
        distance=0    
        if mm < 0:
            self.roomba.drive(-self.drivespeed,0)
        else:
            self.update_position()
            start_pos=np.array([0,0])
            start_pos=self.get_position().copy()
            self.roomba.drive(-self.drivespeed,0)
            while(abs(distance) < abs(mm)):
                self.update_position()
                new_pos=self.get_position()
                #print(distance, start_pos, new_pos)
                distance=np.linalg.norm(new_pos - start_pos)
            self.roomba.drive(0,0)

    def goto(self, destination, tolerance):
        print "Going to ", destination
        while(1):
            position=self.position
            orientation=self.orientation
            v=destination-position
            distance=np.linalg.norm(v)
            if distance < tolerance:
                print "reached ", destination
                return 1 
            theta = math.atan2(v[1],v[0])
            theta=theta-orientation
            if theta < -np.pi:
                theta+=2*np.pi
            elif theta > np.pi:
                theta-=2*np.pi
            print "Position: ", position, " destination: ", destination, " theta ", theta, " distance: ", distance
            self.turn(theta)
            if distance < 1000:
                collisions = self.drive(distance)
            else:
                collisions = self.drive(distance/2 + 100)
            if collisions > 0:
                print "collision"
                self.back(300)
                return 2
    
    def dock(self):
        self.roomba.dock()
        docked=0
        while(docked==0):
            docked=self.is_docked()
            #sleep(0.5)
        self.startfull()
        self.position=np.array([0.0,0.0])
        self.orientation=np.pi*1.5
        self.initial_sensors()
        print("docking")

    def stop(self):
        self.roomba.drive(0,0)
        print "Stopped"

    def startfull(self):
        self.roomba.start()
        self.roomba.full()

    def finish(self):
        self.roomba.dock()
        self.roomba.close()

    def command(self, commands):
        for command in commands:
            self.roomba.cmd(command)

    def initial_sensors(self):
        self.roomba.port.flushInput()
        self.command([149,2,43,44])
        sleep(0.015)
        sensors=self.roomba.port.read(4)
        try:
            self.old_left_encoder=(ord(sensors[0])<<8)|(ord(sensors[1]))
            self.old_right_encoder=(ord(sensors[2])<<8)|(ord(sensors[3]))
        except IndexError: #these operations fail if the roomba has yet to leave the dock
            self.old_left_encoder=0.0
            self.old_right_encoder=0.0

    def update_position(self):

        if self.updateflag ==0: #can only run one update operation at a time
            self.updateflag==1
            self.roomba.port.flushInput()
            self.command([149,4,43,44,45,7])
            sleep(0.015)

            sensors=self.roomba.port.read(6)
            try:
                left_encoder = (ord(sensors[0])<<8)|(ord(sensors[1]))
                right_encoder = (ord(sensors[2])<<8)|(ord(sensors[3]))
                self.ltbumper= 67 & ord(sensors[4])      #todo: convert these to binary, read easier...
                self.crashbumper= 3 & ord(sensors[5])
            except IndexError:
                return -1 
            dl=(left_encoder-self.old_left_encoder)
            dr=(right_encoder-self.old_right_encoder)
            if(dl >= self.encoder_max/2):
                dl-=self.encoder_max
            elif(dl <= -self.encoder_max/2):
                dl+=self.encoder_max
            if (dr >= self.encoder_max/2):
                dr-=self.encoder_max
            elif(dr <= -self.encoder_max/2):
                dr+=self.encoder_max
        
            ddistance = ((0.5*(dl+dr))/self.ticks_rotation)*np.pi*self.wheeldiam
            dangle = (-dl + dr)*(self.wheeldiam*np.pi/self.ticks_rotation)/self.wheelbase
            self.position+=np.array([ddistance*np.cos(self.orientation), ddistance*np.sin(self.orientation)]) 
            self.orientation=(self.orientation+dangle)%(2*np.pi)
            self.old_left_encoder = left_encoder
            self.old_right_encoder = right_encoder
            self.collisions=self.ltbumper|self.crashbumper
            #if self.collisions!=0:
            #    print("collision")
            #    self.roomba.drive(0,0)
            self.updateflag=0
            #self.log()

    def set_position(self,p):
        self.position=p

    def get_position(self):
        return self.position
    
    def get_orientation(self):
        return self.orientation

    def get_collisions(self):
        #1: head on crash
        if self.collisions==0:
            return 0
        elif (self.ltbumper == 12 or self.crashbumper==3): #headon
            return  1
        elif (3&self.ltbumper or 2&self.crashbumper): #left
            return 2
        elif(48&self.ltbumper or 1 & self.crashbumper): #right
            return 3
        else:       #other?? 
            return 4

    def is_docked(self):
        self.command([149,1,21])
        docked=self.roomba.port.read(1)
        docked=3&ord(docked)
        #print("docked?", str(docked))
        return docked

    def recover(self):
        self.roomba.drive(0,0)

def startup():
    """Simple visual startup procedure for debug"""
    print "starting"
    for i in range(0,5):
        led.write(1)
        sleep(0.2)
        led.write(0)
        sleep(0.2)
