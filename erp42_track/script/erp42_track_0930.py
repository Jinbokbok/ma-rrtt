#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import T
import sys,os
import rospy
import rospkg
from std_msgs.msg import Float64,Int16,Float32MultiArray
from control_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg import LocalTrack,LocalControl
from lib.utils_track import purePursuit,pidController
from vehicle_msgs.msg import Waypoint, WaypointsArray
import math
import time
import serial

class erp_planner():
    def __init__(self):
        rospy.init_node('ERP42_track')
        rate = rospy.Rate(5)

        ctrl_msg= CtrlCmd()
        local_track_msg = LocalTrack()
        self.points_msg = WaypointsArray()
        self.curvel_msg = Velocity()
        self.index = 0
        self.target = 150 #40

        pure_pursuit = purePursuit()
        pid = pidController()
        look_steering_point = Waypoint() #steering 계산에 기준이 되는 포인트
        target_velocity = self.target #1m/s

        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        local_track_pub = rospy.Publisher('/local_control', LocalTrack, queue_size=1)
        steer_way_pub = rospy.Publisher('/steer_waypoint', Waypoint, queue_size=1)
        target_velocity_pub = rospy.Publisher('/target_velocity', Velocity, queue_size=1)
        

        rospy.Subscriber("/newwaypoints", WaypointsArray, self.points_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)


        while not rospy.is_shutdown():

            pure_pursuit.getVelStatus(self.curvel_msg)

            ctrl_msg.steering, look_steering_point, avg_steering, steering_ori = pure_pursuit.steering_angle(self.points_msg)
            ctrl_msg.seq += self.index

            print("####################### AVG_STEER : ", avg_steering)
            print("steering? ; ", ctrl_msg.steering)
            print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")

            # if(20 > ctrl_msg.steering > 10 or -20 < ctrl_msg.steering < -10):   ##10, -10
            #     ctrl_msg.brake = 50
            #     target_velocity = 100 #55 130
            # else:
            #     target_velocity = 150 #65

            if(steering_ori > 10 or steering_ori < -10):   ##10, -10
                ctrl_msg.brake = 50
                target_velocity = 130 #55 130
            
            elif (10 >= steering_ori >= 7 or -10 <= steering_ori <= -7):  
                ctrl_msg.brake = 30
                target_velocity = 130 #55 130
            
            elif (7 > steering_ori >= 5 or -7 < steering_ori <= -5):  
                ctrl_msg.brake = 0
                target_velocity = 130 #55 130
            
            else:
                target_velocity = 130 #65

            control_input= pid.pid(self.curvel_msg.velocity, target_velocity)


            if control_input > 0 and control_input <= 200:
                ctrl_msg.accel= target_velocity #*0.1 + self.curvel_msg.velocity
                ctrl_msg.brake= 0

            elif control_input > 200:
                ctrl_msg.accel= target_velocity #*0.05 + self.curvel_msg.velocity
                ctrl_msg.brake= 0

            else :
                print("contol_input else")
                ctrl_msg.accel= 0
                ctrl_msg.brake= -control_input
            
            ctrl_pub.publish(ctrl_msg)

            rospy.loginfo("current_vel : {0}     control_input : {1}".format(self.curvel_msg, control_input))
            # rospy.loginfo("")
            # rospy.loginfo("p_control : {0}     i_control : {1}      d_control : {2}".format(p_control, self.i_control, d_control))
            # rospy.loginfo("error : {0}        output : {1}".format(error, output))

            self.steering_angle=ctrl_msg.steering
            self.control_input=ctrl_msg.accel

            local_track_msg.seq = ctrl_msg.seq
            local_track_msg.velocity = self.curvel_msg.velocity/10
            local_track_msg.steer_angle = self.steering_angle
            local_track_msg.way_x = look_steering_point.x
            local_track_msg.way_y = look_steering_point.y

            local_track_pub.publish(local_track_msg)
            steer_way_pub.publish(look_steering_point)
            target_velocity_pub.publish(target_velocity)
            rate.sleep()

    def points_callback(self,data):
        self.points_msg=WaypointsArray()
        self.points_msg=data

    def velocity_callback(self,speed_data):
        self.curvel_msg=Velocity()
        self.curvel_msg=speed_data

if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass