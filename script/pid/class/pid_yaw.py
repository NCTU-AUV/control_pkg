#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import pid_class
import math
from control_pkg.srv import PidControl, PidControlResponse
import rospy

#       0-3 up/down
#       4-5 forward/backward/left/right
#
#             Front      
#      u                 u
#      0                 3
#       -----------------
#       |               |
#    4  |               |  5
#    f  |               |  f
#       |               |
#       |               |
#       -----------------
#      1                 2
#      u                 u

class Yaw(pid_class.PID):
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setPoint=0.0):
        rospy.init_node('yaw_pid', anonymous=True)
        self.pub = rospy.Publisher('Motors_Force_Yaw', Float64MultiArray, queue_size=10)

        #Coefficient of PID
        #yaw
        self.kp = kp
        self.order_p = 0.0
        self.ki = ki #0.01
        self.order_i = 0.0
        self.kd = kd #0.01
        self.order_d = 0.0
        self.setPoint = setPoint

        self.yaw_pid = pid_class.PID(kp, ki, kd, setPoint)

        #motor_limit
        self.upper_bound = 10
        self.lower_bound = 0.01

        #motor
        self.motor = [0.0]*2
        
        #run
        self.pid_control_server()

    def handle_pid_control(self, req):
        self.kp = req.p
        self.order_p = req.po
        self.ki = req.i
        self.order_i = req.io
        self.kd = req.d
        self.order_d = req.do

        print("Get control msg [%f %f %f %f %f %f]"%(self.kp, self.order_p, self.ki, self.order_i, self.kd, 
        self.order_d))

        self.yaw_pid.setAllCoeff([self.kp * pow(10, self.order_p), self.ki * pow(10, self.order_i), self.kd * pow(10, self.order_d)])
        return PidControlResponse(True)

    def pid_control_server(self):
        # rospy.init_node('pid_control_server')
        self.s = rospy.Service('yaw_pid_control', PidControl, self.handle_pid_control)
        #print("Ready to get control msg.")
        #rospy.spin()

    def update_motor(self, force):
        self.motor = [force, -force]

if __name__ == '__main__':
    yaw = Yaw(1.5, 0, 0, 0)
