#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pid_class
import math
from control_pkg.srv import PidControl, PidControlResponse
import rospy

#       0-3 up/down
#       4-5 thrust
#       6-7 direction
#               
#      u       <-        u
#      0        7        3
#       -----------------
#       |               |
#    4  |               |  5
#    f  |depth0   depth1|  f
#       |               |
#       |               |
#       -----------------
#      1        6        2
#      u       ->        u

class Depth:

    def __init__(self, kp_l, ki_l, kd_l, kp_r, ki_r, kd_r):
        rospy.init_node('depth_pid', anonymous=True)
        self.pub = rospy.Publisher('Motors_Force_Depth', Float64MultiArray, queue_size=10)
        
        #Coefficient of PID
        #depth_left
        self.kp_l = kp_l
        self.order_p_l = 4
        self.ki_l = ki_l #0.01
        self.order_i_l = 0
        self.kd_l = kd_l #0.01
        self.order_d_l = 0

        #depth_right
        self.kp_r = kp_r
        self.order_p_r = 4
        self.ki_r = ki_r #0.01
        self.order_i_r = 0
        self.kd_r = kd_r #0.01
        self.order_d_r = 0

        self.depth_pid_l = pid_class.PID(kp_l, ki_l, kd_l)
        self.depth_pid_r = pid_class.PID(kp_r, ki_r, kd_r)

        #motor_limit
        self.upper_bound = 10000
        self.lower_bound = 0.01

        #set depth
        self.depth = 0
        self.depth_pid_l.setSetPoint(self.depth)
        self.depth_pid_r.setSetPoint(self.depth)

        #motor
        self.motor = [0.0]*4

        #run
        self.pid_control_server()
        self.listener()

    def handle_pid_control(self, req):
        #set coefficient of pid for left
        self.kp_l = req.p_l
        self.order_p_l = req.po_l
        self.ki_l = req.i_l
        self.order_i_l = req.io_l
        self.kd_l = req.dl
        self.order_d_l = req.do_l

        #set coefficient of pid for right
        self.kp_r = req.p_r
        self.order_p_r = req.po_r
        self.ki_r = req.i_r
        self.order_i_r = req.io_r
        self.kd_r = req.d_r
        self.order_d_r = req.do_r

        print("Get control msg [%f %f %f %f %f %f %f %f %f %f %f %f]"%(self.kp_l, self.order_p_l, self.ki_l, self.order_i_l, self.kd_l, 
        self.order_d_l, self.kp_r, self.order_p_r, self.ki_r, self.order_i_r, self.kd_r, self.order_d_r))

        self.depth_pid_l.setAllCoeff([self.kp_l, self.ki_l, self.kd_l])
        self.depth_pid_r.setAllCoeff([self.kp_r, self.ki_r, self.kd_r]) 

        return PidControlResponse(True)

    def pid_control_server(self):
        # rospy.init_node('pid_control_server')
        self.s = rospy.Service('depth_pid_control', PidControl, self.handle_pid_control)
        #print("Ready to get control msg.")
        rospy.spin()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
        feedback = [self.depth_pid_l.update_Feedback(data.data[0]), 
                    self.depth_pid_r.update_Feedback(data.data[1])]
        #print(feedback)
        self.update_motor(feedback)

        self.talker()

    def listener(self):
        rospy.Subscriber("IMU/Depth", Float64MultiArray, self.callback)
        rospy.spin()

    def update_motor(self, force):
        value_depth_l = force[0]
        value_depth_r = force[1]

        self.value = [value_depth_l, value_depth_l, value_depth_r, value_depth_r]

        for i in range(4):
            if self.value[i] < -self.upper_bound:
                self.motor[i] = -self.upper_bound
            elif self.value[i] > self.upper_bound:
                self.motor[i] = self.upper_bound
            elif self.value[i] > -self.lower_bound and self.value[i] < self.lower_bound:
                self.motor[i] = 0
            else:
                self.motor[i] = self.value[i]

    def talker(self):
        rospy.loginfo(self.motor)
        self.pub.publish(Float64MultiArray(data = self.motor))
       
if __name__ == '__main__':
    depth = Depth(1, 0, 0, 1, 0, 0)
