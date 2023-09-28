#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32, Float32 ,Float64
from control_msgs.msg import Velocity

from morai_msgs.msg import CtrlCmd

from math import *

import math
import time
import serial
import struct

rospy.set_param('PORT', '/dev/ttyUSB0')

S = chr(0x53)
T = chr(0x54)
X = chr(0x58)
AorM = chr(0x00)
ESTOP = chr(0x00)
GEAR = 0x00
SPEED0 = 0x00
SPEED1 = 0x00
STEER0 = 0x00
STEER1 = 0x00
BRAKE = 0x00
ENC0 = 0x00
ENC1 = 0x00
ENC2 = 0x00
ENC3 = 0x00
ALIVE = chr(0x00)
ETX0 = chr(0x0D)
ETX1 = chr(0x0A)

count=0
count_alive=0

packet=[]
speed = 0
gear = 0x00
brake = 0
steer = 0
encoder = 0x00

##########Send to ERP42###########################################################

def GetAorM():
    AorM = chr(0x01)
    return  AorM

def GetESTOP():
    ESTOP = chr(0x00)
    return  ESTOP

def GetGEAR(gear_s):
    GEAR = chr(gear_s)
    return  GEAR

def GetSPEED(speed_s):
    global count
    SPEED0 = chr(0x00)
    SPEED1 = chr(int(speed_s))
    print(SPEED0)
    #print(SPEED1)
    return SPEED0, SPEED1

def GetSTEER(steer_s):
    steer_s=steer_s*71
    steer_max=0b0000011111010000#+2000
    steer_0 = 0b0000000000000000
    steer_min=0b1111100000110000#-2000

    #print(steer)
    if (steer_s>=0 and steer_s<=2000):
        angle=int(steer_s)
        STEER=steer_0+angle
    elif (steer_s>2000):
        STEER=int(steer_max)
    elif (steer_s<-2000):
        STEER=int(steer_min)
    else:
        angle=int(-steer_s)
        angle=2000-angle
        STEER=steer_min+angle

    STEER0=STEER & 0b1111111100000000
    STEER0=STEER0 >> 8
    STEER1=STEER & 0b0000000011111111

    return chr(STEER0), chr(STEER1)

def GetBRAKE(brake_s):
    BRAKE = chr(int(brake_s))
    return  BRAKE


def Send_to_ERP42(gear_s, speed_s, steer_s, brake_s):
    global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive

    count_alive = count_alive+1

    if count_alive==0xff:
        count_alive=0x00

    AorM = GetAorM()
    ESTOP = GetESTOP()
    GEAR = GetGEAR(gear_s)
    SPEED0, SPEED1 = GetSPEED(speed_s)
    STEER0, STEER1 = GetSTEER(steer_s)
    BRAKE = GetBRAKE(brake_s)

    ALIVE = chr(count_alive)
    vals = [S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]

    for i in range(len(vals)):
        ser.write(vals[i]) #send!

########## CONVERTER ###############

#def Gear_Conv(gear_b):
#    gear = 0
#    gear = (bytearray(gear_b))
#
#    print('gear : ' , gear)
#    return gear[0]

def Speed_Conv(speed0_b,speed1_b):
    speed = 0
    speed0 = (bytearray(speed0_b))
    speed1 = (bytearray(speed1_b))

    speed = speed0[0] + 256*speed1[0]

    print('speed : ', speed)
    return speed

def Steer_Conv(steer0_b,steer1_b):
    steer = 0
    steer0 = (bytearray(steer0_b))
    steer1 = (bytearray(steer1_b))

    if(steer1[0] & 0b00001000) == 0:
        steer = -(steer0[0] + 256*steer1[0])
    elif (steer1[0] & 0b0001000) == 8:
        steer = -(-2047 + (steer0[0] + 256*(steer1[0] & 0b00000111)))

    #print(steer) # 1 is 0.014 degree, so 71 is 1 degree

    print('steer : ', steer)
    return steer

def Brake_Conv(brake_b):
    brake = 0
    brake = (bytearray(brake_b))

    return brake[0]

def Encoder_Conv(enc0_b,enc1_b,enc2_b,enc3_b):
    encoder = 0x00
    encoder0 = (bytearray(enc0_b))
    encoder1 = (bytearray(enc1_b))
    encoder2 = (bytearray(enc2_b))
    encoder3 = (bytearray(enc3_b))

    if(encoder3[0] & 0x80000000) == 0:
        encoder = encoder0[0] + (2**8)*encoder1[0] + (2**16)*encoder2[0] + (2**24)*encoder3[0]
    else:
        encoder = -2**32 + 1 +(encoder0[0] + (2**8)*encoder1[0] + (2**16)*encoder2[0] + (2**24)*(encoder3[0] & 0x7fffffff))

    print('encoder : ' , encoder)
    return encoder



########## DECODER Send to PC###############

def Parsing():
    global S,T,X,AorM,ESTOP,GEAR,SPEED0,SPEED1,STEER0,STEER1,BRAKE,ENC0,ENC1,ENC2,ENC3,ALIVE,ETX0,ETX1,speed,gear,brake,steer,encoder,packet

    Pdata = []
    Pdata = ser.readline()

    if(len(Pdata) == 18):
        STEER0 = Pdata[8:9]
        STEER1 = Pdata[9:10]
        steer = Steer_Conv(STEER0,STEER1)

        #GEAR = Pdata[5:6]
        #gear = Gear_Conv(GEAR)

        SPEED0 = Pdata[6:7]
        SPEED1 = Pdata[7:8]
        speed = Speed_Conv(SPEED0, SPEED1)

        BRAKE = Pdata[10:11]
        brake = Brake_Conv(BRAKE)

        ENC0 = Pdata[11:12]
        ENC1 = Pdata[12:13]
        ENC2 = Pdata[13:14]
        ENC3 = Pdata[14:15]
        encoder = Encoder_Conv(ENC0,ENC1,ENC2,ENC3)
        print("encoder : " , ENC0, ENC1, ENC2, ENC3)

gear_s = 0
speed_s = 0
steer_s = 0
brake_s = 0

def cmd_callback(data):
    global gear_s, speed_s, steer_s, brake_s

    #gear_s = data.gear
    speed_s = data.accel #speed
    steer_s = data.steering
    brake_s = data.brake
    print(steer_s)


if __name__ == '__main__':
    rospy.init_node('ERP42_status') #
    pub_encoder_erp42 = rospy.Publisher('/ERP42_encoder', Float64, queue_size=10)
    pub_steer_erp42 = rospy.Publisher('/ERP42_steer', Float32, queue_size=10)
    pub_steer_erp42_conv = rospy.Publisher('/ERP42_steer_conv', Float32, queue_size=10)
    pub_speed_erp42 = rospy.Publisher('/ERP42_velocity', Velocity, queue_size=10)

    # 220520
    pub_speed_s_erp42 = rospy.Publisher('/ERP42_velocity_s', Velocity, queue_size=10)
    pub_steer_s_erp42 = rospy.Publisher('/ERP42_steer_s', Float32, queue_size=10)
    pub_brake_s_rp42 = rospy.Publisher('/ERP42_brake_s', Float32, queue_size=10)
    #pub_target_speed_erp42 = rospy.Publisher('/ERP42_target_velocity', Float32, queue_size=10)


    #pub_gear_erp42 = rospy.Publisher('/ERP42_gear', Float32, queue_size=10)
    pub_brake_erp42 = rospy.Publisher('/ERP42_brake', Float32, queue_size=10)
    rospy.Subscriber('/ctrl_cmd',CtrlCmd,cmd_callback)
    rate = rospy.Rate(20) #

    ser = serial.Serial(rospy.get_param('PORT'),
                        baudrate=115200,
                        timeout=None,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE
                        )

    while (ser.isOpen() and (not rospy.is_shutdown())):

        Parsing()
        #print('packet : ' + packet)

        pub_speed_erp42.publish(speed)

        pub_encoder_erp42.publish(encoder)
        pub_steer_erp42.publish(steer)
        pub_steer_erp42_conv.publish(steer/71)
        pub_speed_s_erp42.publish(speed_s)
        pub_steer_s_erp42.publish(steer_s)
        pub_brake_s_rp42.publish(brake_s)
        #pub_target_speed_erp42.publish(140)
        
        #pub_gear_erp42.publish(gear)
        pub_brake_erp42.publish(brake)

        print("status 1 : ",speed, steer, brake)
        print("status 2 : ", speed_s, steer_s, brake_s)
        
        Send_to_ERP42(gear_s, speed_s, steer_s, brake_s)
        #print(speed_s) #Send to ERP42
