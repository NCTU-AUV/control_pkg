#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pid_class

class Motor_Listener:
    def __init__(self):
        self.motor = [0.0]*8
        self.motor_attitude = [0.0]*4
        self.motor_depth = [0.0]*4

        inval = float(input('Input DC Force (e.g 2): '))
        self.dc_force = [inval for _ in range(4)]
        
        rospy.init_node('merge_motor', anonymous=True)
        self.pub = rospy.Publisher('Motors_Force', Float64MultiArray, queue_size=10)
        rospy.Subscriber('Motors_Force_Attitude', Float64MultiArray, self.callback_attitude)
        rospy.Subscriber('Motors_Force_Depth', Float64MultiArray, self.callback_depth)  
    def callback_attitude(self, data):
        print(f'Attitude Force {data.data}')
        for i in range(4):
            self.motor_attitude[i] = data.data[i]        

    def callback_depth(self, data):
        for i in range(4):
            self.motor_depth[i] = data.data[i]

    def talker(self):
        for i in range(4):
            self.motor[i] = self.motor_attitude[i] + self.motor_depth[i] + self.dc_force[i]
        self.pub.publish(Float64MultiArray(data=self.motor))

if __name__ == '__main__':
    motor = Motor_Listener()
    rate = rospy.Rate(10)   
    try:
        while not rospy.is_shutdown():
            motor.talker()
            rate.sleep()
    except KeyboardInterrupt:
        print('bye')
