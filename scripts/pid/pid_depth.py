#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float64, Int16
import math
#from control_pkg.srv import PidControl, PidControlResponse
import rospy
from Class import pid_class

#       0-3 up/down
#       4-5 forward/backward/left/right
#
#             Front 
#      u                 u
#      0                 3
#       -----------------
#       |               |
#    4  |               |  5
#    f  |depth0   depth1|  f
#       |               |
#       |               |
#       -----------------
#      1                 2
#      u                 u

class Depth:

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, depth=0.0, const_force=0.82):
        rospy.init_node('depth_pid', anonymous=True)
        self.pub = rospy.Publisher('Motors_Force_Depth', Float64MultiArray, queue_size=10)
        self.pub1 = rospy.Publisher('setpoint/depth', Float64, queue_size=10)
        
        #Coefficient of PID
        self.depth_pid = pid_class.PID(kp, ki, kd, depth)

        self.value = 0
        self.const_force = const_force
        
        #motor_limit
        self.upper_bound = 10
        self.lower_bound = 0.01

        #set depth
        self.depth = depth

        #on/off
        self.on_depth = 1

        #motor
        self.motor = [0.0]*4

        #run
        #self.pid_control_server()
        print('start to run')
        rospy.Subscriber("Depth", Float64, callback=self.callback_depth)
        rospy.Subscriber("On/Depth", Int16, callback=self.callback_on)
        rospy.spin()

    #def handle_pid_control(self, req):
        #set coefficient of pid
        #self.kp = req.p
        #self.order_p = req.po
        #self.ki = req.i
        #self.order_i = req.io
        #self.kd = req.dl
        #self.order_d = req.do

        #print("Get control msg [%f %f %f %f %f %f]"%(self.kp, self.order_p, self.ki, self.order_i, self.kd, self.order_d))

        #self.depth_pid.setAllCoeff([self.kp, self.ki, self.kd])
        #self.depth_pid.setAllCoeff([self.kp * pow(10, self.order_p), self.ki * pow(10, self.order_i), self.kd * pow(10, self.order_d)]) 

        #return PidControlResponse(True)

    #def pid_control_server(self):
        # rospy.init_node('pid_control_server')
        #self.s = rospy.Service('depth_pid_control', PidControl, self.handle_pid_control)
        #print("Ready to get control msg.")
        #rospy.spin()

    def callback_depth(self, data):
        #print(data.data)
        #rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
        feedback = self.depth_pid.update_Feedback(data.data)
        #print(feedback)
        self.update_motor(feedback)

        if not self.on_depth:
            rospy.set_param('/Bar30/depth', data.data)

        self.talker()
    
    def callback_on(self, data):
        if (self.on_depth ^ data.data) and data.data:
            self.depth = rospy.get_param("/Bar30/depth")
            self.depth_pid.setSetPoint(self.depth)
            self.pub1.publish(self.depth)

        self.on_depth = data.data

    def update_motor(self, force):
        self.value = force * self.on_depth + self.const_force
        #print(force)

        for i in range(4):
            if self.value < -self.upper_bound:
                self.motor[i] = -self.upper_bound
            elif self.value > self.upper_bound:
                self.motor[i] = self.upper_bound
            elif self.value > -self.lower_bound and self.value < self.lower_bound:
                self.motor[i] = 0
            else:
                self.motor[i] = self.value

    def talker(self):
        print([f'{_:.2f}' for _ in self.motor])
        self.pub.publish(Float64MultiArray(data = self.motor))
       
if __name__ == '__main__':
    depth = Depth(0.05, 0.001, 0.003, 50)
#0.04 0 0.0025 40
#0.05 0.001 0.003 40