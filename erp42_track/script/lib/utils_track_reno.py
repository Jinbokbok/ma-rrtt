#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import rospkg
from geometry_msgs.msg import Point
from std_msgs.msg import Float64,Int16,Float32MultiArray, Int32, Float32
#import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
from vehicle_msgs.msg import Waypoint, WaypointsArray
from math import *

import math
import time
import serial

new_waypoint = Waypoint()

class purePursuit :
    def __init__(self):
        self.forward_point=Waypoint()
        self.vehicle_length=1.04 # 제원표는 1.6 => 물어보기
        self.steering=0
        self.avg_steering = 0

    def getVelStatus(self,msg):
        self.current_vel=msg.velocity

#######Local Path#######
    def steering_angle(self, newpoints):

        if (len(newpoints.waypoints) <= 0) :
            self.steering = 0
            return self.steering, self.forward_point, self.avg_steering

        self.forward_point = newpoints.waypoints[0]

        if(len(newpoints.waypoints) >= 3):
            self.avg_steering = (newpoints.waypoints[0].x + newpoints.waypoints[1].x+ newpoints.waypoints[2].x) / 3

        dx = self.forward_point.x
        dy = self.forward_point.y
        dis = math.sqrt(dx*dx + dy*dy)
        alpha = atan(dy/dx) #radian
        delta = 2 * self.vehicle_length * sin(alpha)/dis
        self.steering= -atan(delta)*180/pi #deg
        rospy.loginfo("target point : {0}        {1}".format(self.forward_point.x, self.forward_point.y))
        rospy.loginfo("target steer : {0}".format(self.steering))
        return self.steering, self.forward_point, self.avg_steering

class pidController : ## 속도 제어를 위한 PID 적용 ##
    def __init__(self):
        self.p_gain=1 
        self.i_gain=4 #0.5   #steady state error
        self.d_gain=0.35 #0.8 #overshoot

        self.controlTime=0.1
        self.prev_error=0
        self.i_control=0


    def pid(self, current_vel, target_velocity):
        error= target_velocity-current_vel

        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error

        return output
