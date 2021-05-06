#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float64MultiArray
import pid_class

class Motor_Listener:
    def __init__(self):
        self.motor = [0.0]*6
        self.motor_attitude = [0.0]*6
        self.motor_depth = [0.0]*6
        self.motor_dc = [0.0]*6
        self.motor_command = [0.0]*6
        self.coe = [1.11, -1.01, 1.42, -1.21, 1.50, -1] #motor 0 and motor 2 3 reverse   
        
        rospy.init_node('merge_motor', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher('Motors_Force', Float64MultiArray, queue_size=10)
        
        rospy.Subscriber('Motors_Force_Attitude', Float64MultiArray, self.callback_attitude)
        rospy.Subscriber('Motors_Force_Depth', Float64MultiArray, self.callback_depth)  
        rospy.Subscriber('Motors_Force_Command', Float64MultiArray, self.callback_command)    

    def callback_attitude(self, data):
        #print(f'Attitude Force {data.data}')
        for i in range(6):
            self.motor_attitude[i] = data.data[i]        

    def callback_depth(self, data):
        for i in range(4):
            self.motor_depth[i] = data.data[i]

    def callback_command(self, data):
        for i in range(6):
            self.motor_command[i] = data.data[i]

    def talker(self):
        for i in range(6):
            self.motor[i] = self.motor_attitude[i] + self.motor_depth[i] + self.motor_command[i]
            self.motor[i] *= self.coe[i]
        print(self.motor)
        self.pub.publish(Float64MultiArray(data=self.motor))

    def shutdown(self):
        for _ in range(3):
            self.pub.publish(Float64MultiArray(data=[0.0]*6))
            time.sleep(0.001)            
        print('set motor array to zero')        

if __name__ == '__main__':
    motor = Motor_Listener()
    rate = rospy.Rate(10)   
    try:
        while not rospy.is_shutdown():
            motor.talker()
            rate.sleep()
    except KeyboardInterrupt:
        print('bye')
