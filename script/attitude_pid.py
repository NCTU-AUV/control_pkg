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
#    f  |               |  f
#       |               |
#       |               |
#       -----------------
#      1        6        2
#      u       ->        u

class Attitude:
    
    def __init__(self, kp_r=1.0, ki_r=0.0, kd_r=0.0, kp_p=1.0, ki_p=0.0, kd_p=0.0):     
        rospy.init_node('attitude_pid', anonymous=True)
        self.pub = rospy.Publisher('Motors_Force_Attitude', Float64MultiArray, queue_size=10)

        #Coefficient of PID
        #roll
        self.kp_r = kp_r
        self.order_p_r = -2
        self.ki_r = ki_r #0.01
        self.order_i_r = 0
        self.kd_r = kd_r #0.01
        self.order_d_r = 0
        self.setPoint_r = -10

        #pitch
        self.kp_p = kp_p
        self.order_p_p = -2
        self.ki_p = ki_p #0.01
        self.order_i_p = 0
        self.kd_p = kd_p #0.01
        self.order_d_p = 0
        self.setPoint_p = 10

        self.roll_pid = pid_class.PID(kp_r, ki_r, kd_r, self.setPoint_r)
        self.pitch_pid = pid_class.PID(kp_p, ki_p, kd_p, self.setPoint_p)

        #motor_limit
        self.upper_bound = 10000
        self.lower_bound = 0.01

        #motor
        self.motor = [0.0]*4
        
        #run
        self.pid_control_server()
        self.listener()

    def handle_pid_control(self, req):
        self.kp_r = req.p_r
        self.order_p_r = req.po_r
        self.ki_r = req.i_r
        self.order_i_r = req.io_r
        self.kd_r = req.d_r
        self.order_d_r = req.do_r
        self.kp_p = req.p2_p
        self.order_p_p = req.po_p
        self.ki_p = req.i_p
        self.order_i_p = req.io_p
        self.kd_p = req.d_p
        self.order_d_p = req.do_p

        print("Get control msg [%f %f %f %f %f %f %f %f %f %f %f %f]"%(self.kp_r, self.order_p_r, self.ki_r, self.order_i_r, self.kd_r, 
        self.order_d_r, self.kp_p, self.order_p_p, self.ki_p, self.order_i_p, self.kd_p, self.order_d_p))

        self.roll_pid.setAllCoeff([self.kp_r, self.ki_r, self.kd_r])
        self.pitch_pid.setAllCoeff([self.kp_p, self.ki_p, self.kd_p])

        return PidControlResponse(True)

    def pid_control_server(self):
        # rospy.init_node('pid_control_server')
        self.s = rospy.Service('attitude_pid_control', PidControl, self.handle_pid_control)
        #print("Ready to get control msg.")
        #rospy.spin()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
        feedback = [self.roll_pid.update_Feedback(data.data[0]), self.pitch_pid.update_Feedback(data.data[1])]
        #print(feedback)
        self.update_motor(feedback)

        self.talker()

    def listener(self):
        rospy.Subscriber("IMU/Attitude", Float64MultiArray, self.callback)
        rospy.spin()

    def update_motor(self, force):
        value_roll = force[0]
        value_pitch = force[1]
        #self.value = [value_roll, value_roll, -value_roll, -value_roll]
        #self.value = [value_pitch, -value_pitch, -value_pitch, value_pitch]
        self.value = [value_roll+value_pitch, value_roll-value_pitch, 
        -value_roll-value_pitch, -value_roll+value_pitch]

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
    attitude = Attitude(150, -0.05, 0.05, 100, 0, 0)
