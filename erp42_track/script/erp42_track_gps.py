#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import T
import sys,os
import rospy
import rospkg

from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray
from control_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg import LocalTrack,LocalControl
from lib.utils_track_gps import purePursuit,purePursuitGPS,pidController,findLocalPath
from morai_msgs.msg import CtrlCmd, LocalControl
from ublox_msgs.msg import NavPVT
from vehicle_msgs.msg import Waypoint, WaypointsArray
import math
import time
import serial

import threading
import datetime

class erp_planner():
    def __init__(self):
        rospy.init_node('ERP42_track')
        rate = rospy.Rate(5)

        ctrl_msg= CtrlCmd()
        local_track_msg = LocalTrack()
        self.points_msg = WaypointsArray()
        self.curvel_msg = Velocity()
        self.index = 0
        self.target = 50
        self.save_waypoint = Path() # save waypoint
        self.isStart = False
        self.start_gps_value = ()
        self.current_gps = ()
        self.lab = 0
        self.dis = 0.0
        self.lab2 = 0
        self.start = time.time()
        self.lab_time = time.time()

        pure_pursuit = purePursuit()
        pure_pursuitGPS = purePursuitGPS()
        self.pose_msg=Odometry()
        self.curvel_msg=Velocity()
        self.yaw_msg=NavPVT()
        pid = pidController()
        look_steering_point = Waypoint() #steering 계산에 기준이 되는 포인트
        target_velocity = self.target #1m/s
        self.current_waypoint = 0
        self.target_angle_index = 0

        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        local_track_pub = rospy.Publisher('/local_control', LocalTrack, queue_size=1)
        steer_way_pub = rospy.Publisher('/steer_waypoint', Waypoint, queue_size=1)
        target_velocity_pub = rospy.Publisher('/target_velocity', Velocity, queue_size=1)
        control_velocity_pub = rospy.Publisher('/control_input', Velocity, queue_size=1)
        track_gps_waypoint_pub = rospy.Publisher('/newGPSwaypoints',Path, queue_size=1)
        

        rospy.Subscriber("/newwaypoints", WaypointsArray, self.points_callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.yaw_callback)
        rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
        rospy.Subscriber("/odom/filtered", Odometry, self.gps_callback)

        # GPS VALUE 가져와서 저장해야하고 ->

        while not rospy.is_shutdown():
            print("#################### LAB : ", self.lab, "LAB2 : ", self.lab2, "##########################")
            print("#################### LAB_Time : ", self.lab_time, "##########################")

            pure_pursuit.getVelStatus(self.curvel_msg)

            if self.lab < 2 :
                self.target = 40
                self.save_gps()
                ctrl_msg.steering, look_steering_point, previous_point, self.dis = pure_pursuit.steering_angle(self.points_msg)
                ctrl_msg.seq += self.index

                control_input = pid.pid(self.curvel_msg.velocity, target_velocity)

                if control_input > 0 and control_input <= 200:
                    ctrl_msg.accel= target_velocity #*0.1 + self.curvel_msg.velocity
                    ctrl_msg.brake= 0

                elif control_input > 200:
                    ctrl_msg.accel= target_velocity #*0.05 + self.curvel_msg.velocity
                    ctrl_msg.brake= 0

                else :
                    print("contol_input else")
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -target_velocity
            else:
                self.target = 70    
                local_path, self.current_waypoint=findLocalPath(self.save_waypoint, self.pose_msg, self.current_waypoint)

                #pure_pursuit
                pure_pursuitGPS.getPath(self.save_waypoint)
                pure_pursuitGPS.getPoseStatus(self.pose_msg)
                pure_pursuitGPS.getVelStatus(self.curvel_msg)
                pure_pursuitGPS.getYawStatus(self.yaw_msg)
                
                ctrl_msg.steering, self.target_angle_index, self.nofoundpoint = pure_pursuitGPS.steering_angle(self.current_waypoint)
                ctrl_msg.seq += self.index
                
                control_input = pid.pid(self.curvel_msg.velocity, target_velocity)

                if control_input > 0 and control_input <= 200:
                    ctrl_msg.accel= control_input #*0.1 + self.curvel_msg.velocity
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

            # 한 바퀴 안에서만 look_steering_point 저장
            # print("===========================", look_steering_point.x, ", " , look_steering_point.y, ", " , self.steering_angle, "==================")
            print("============================ ABS ", look_steering_point == previous_point,"==================================")
        
            local_track_pub.publish(local_track_msg)
            steer_way_pub.publish(look_steering_point)
            target_velocity_pub.publish(target_velocity)
            control_velocity_pub.publish(control_input)
            track_gps_waypoint_pub.publish(self.save_waypoint)
            rate.sleep()
    
    def pose_callback(self,data):
        self.pose_msg=Odometry()
        self.pose_msg=data


    def velocity_callback(self,speed_data):
        self.curvel_msg=Velocity()
        self.curvel_msg=speed_data


    def yaw_callback(self,yaw_data):
        self.yaw_msg=NavPVT()
        self.yaw_msg=yaw_data


    def points_callback(self,data):
        self.points_msg=WaypointsArray()
        self.points_msg=data
    
    def gps_callback(self,data):
        self.current_gps = ()
        self.current_gps = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

        if(not self.isStart):
            self.lab = 1
            self.isStart = True
            self.start_gps_value = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

    def save_gps(self):
        if self.isStart :
            # local 모드 일때
            error_x = self.start_gps_value[0] - self.current_gps[0]
            error_y = self.start_gps_value[1] - self.current_gps[1]
            print(self.start_gps_value)
            print(self.current_gps)
            print("==================================== (", error_x,",",error_y,") ================================")
            
            if(-0.1 < error_x <= 0.1 and -0.1 < error_y <= 0.1):
                self.lab_time = time.time() - self.start
                print("*************************** LAB TIME ::: ", self.lab_time)
                if(self.lab_time > 15):
                    self.lab += 1

            if(error_y == 0.0 and error_x == 0.0):
                self.lab2 += 1
            
            # gps 데이터 저장
            self.save_waypoint.header.frame_id='map'
            tmp_pose=PoseStamped()
            tmp_pose.pose.position.x= self.current_gps[0]
            tmp_pose.pose.position.y= self.current_gps[1]
            tmp_pose.pose.position.z= self.current_gps[2]
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            self.save_waypoint.poses.append(tmp_pose)

    def velocity_callback(self,speed_data):
        self.curvel_msg=Velocity()
        self.curvel_msg=speed_data


if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()

    except rospy.ROSInterruptException:
        pass
