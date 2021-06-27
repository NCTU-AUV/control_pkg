#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Int16
import math
#from control_pkg.srv import PidControl, PidControlResponse
import rospy
#from Class import pid_roll, pid_pitch, pid_yaw
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
#    f  |               |  f
#       |               |
#       |               |
#       -----------------
#      1                 2
#      u                 u

class Attitude:
    
    def __init__(self, kp_r=1.0, ki_r=0.0, kd_r=0.0, setPoint_r=0.0, kp_p=1.0, ki_p=0.0, kd_p=0.0, setPoint_p=0.0, 
    kp_y=1.0, ki_y=0.0, kd_y=0.0, setPoint_y=0.0):
        rospy.init_node('attitude_pid', anonymous=True)
        self.pub = rospy.Publisher('Motors_Force_Attitude', Float64MultiArray, queue_size=10)

        #Coefficient of PID
        #self.roll_pid = pid_roll.Roll(kp_r, ki_r, kd_r, setPoint_r)
        #self.pitch_pid = pid_pitch.Pitch(kp_p, ki_p, kd_p, setPoint_p)
        #self.yaw_pid = pid_yaw.Yaw(kp_y, ki_y, kd_y, setPoint_y)
        self.roll_pid = pid_class.PID(kp_r, ki_r, kd_r, setPoint_r)
        self.pitch_pid = pid_class.PID(kp_p, ki_p, kd_p, setPoint_p)
        self.yaw_pid = pid_class.PID(kp_y, ki_y, kd_y, setPoint_y)


        #motor_limit
        self.upper_bound = 10
        self.lower_bound = 0.01

        #motor
        self.motor = [0.0]*6
        
        #on/off
        self.on_attitude = 1

        #run
        #self.pid_control_server()
        print('start to run')
        rospy.Subscriber("IMU/Attitude", Float64MultiArray, self.callback_attitude)
        rospy.Subscriber("On/Attitude", Int16, self.callback_on)
        rospy.spin()

    #def handle_pid_control(self, req):
        #print("Get roll msg [%f %f %f %f %f %f]" %(req.p_r, req.po_r, req.i_r, req.io_r, req.d_r, req.do_r))
        #print("Get pitch msg [%f %f %f %f %f %f]" %(req.p_p, req.po_p, req.i_p, req.io_p, req.d_p, req.do_p))
        #print("Get yaw msg [%f %f %f %f %f %f]" %(req.p_y, req.po_y, req.i_y, req.io_y, req.d_y, req.do_y))

        #self.roll_pid.setAllCoeff([req.p_r * pow(10, req.po_r), req.i_r * pow(10, req.io_r), req.d_r * pow(10, req.do_r)])
        #self.pitch_pid.setAllCoeff([req.p_p * pow(10, req.po_p), req.i_p * pow(10, req.io_p), req.d_p * pow(10, req.do_p)])
        #self.yaw_pid.setAllCoeff([req.p_y * pow(10, req.po_y), req.i_y * pow(10, req.io_y), req.d_y * pow(10, req.do_y)])

        #return PidControlResponse(True)

    #def pid_control_server(self):
        # rospy.init_node('pid_control_server')
        #self.s = rospy.Service('attitude_pid_control', PidControl, self.handle_pid_control)
        #print("Ready to get control msg.")
        #rospy.spin()

    def callback_attitude(self, data):
        #\rospy.loginfo(rospy.get_caller_id() + "%s", data.data)
        feedback = [self.roll_pid.update_Feedback(data.data[1]), self.pitch_pid.update_Feedback(data.data[0])
                   , self.yaw_pid.update_Feedback(data.data[2])]
        
        if not self.on_attitude:
            rospy.set_param('/IMU/yaw', data.data[2])

        self.update_motor(feedback)
        
        # self.roll_pid.update_motor(feedback[0])
        # self.pitch_pid.update_motor(feedback[1])
        # self.yaw_pid.update_motor(feedback[2])

        self.talker()

    def callback_on(self, data):
        if (self.on_attitude ^ data.data) and data.data:
            self.yaw_pid.setSetPoint(rospy.get_param("/IMU/yaw"))
            
        self.on_attitude = data.data

    def update_motor(self, force):
        value_roll = force[0]
        value_pitch = force[1]
        value_yaw = force[2]
        
        self.value = [value_roll+value_pitch, value_roll-value_pitch, 
        -value_roll-value_pitch, -value_roll+value_pitch, -value_yaw * self.on_attitude, value_yaw * self.on_attitude]

        for i in range(6):
            if self.value[i] < -self.upper_bound:
                self.motor[i] = -self.upper_bound
            elif self.value[i] > self.upper_bound:
                self.motor[i] = self.upper_bound
            elif self.value[i] > -self.lower_bound and self.value[i] < self.lower_bound:
                self.motor[i] = 0
            else:
                self.motor[i] = self.value[i]

    def talker(self):
        print([f'{_:.2f}' for _ in self.motor])
        self.pub.publish(Float64MultiArray(data = self.motor))

if __name__ == '__main__':
    attitude = Attitude(0.013, 0.001, 0.01, 0.12, 0.013, 0.001, 0.01, 0.1, 0.0, 0.0, 0.0, 293)
#0.013 0.0001 0.01