#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64MultiArray
import math
import time
from control_pkg.srv import PidControl, PidControlResponse
import attitude_pid
import depth_pid

class Controller():
    def __init__(self, command = 'stop'):
        rospy.init_node('Controller', anonymous=True)
        self.pub_motor = rospy.Publisher('Motors_Force_Command', Float64MultiArray, queue_size=10)

        self.command = command
        self.motor = [0.0]*6
        
        print('ready to controll vehicle')

        self.listener()
    
    def listener(self):
        rospy.Subscriber('cmd', String, self.callback)
        rospy.Subscriber('tracking_command', String, self.callback)
        rospy.spin()

    def callback(self, data):
        self.command = data.data
        self.switch(data.data)
        self.talker()

    def switch(self, command):
        return{
            'stop': self.stop,
            'up': self.move_vertical,
            'down': self.move_vertical,
            'forward': self.move_horizontal,
            'backward': self.move_horizontal,
            'left': self.move_horizontal,
            'right': self.move_horizontal
        }[command](command)

    def stop(self, command):
        for i in range(6):
            self.motor[i] = 0.0
    
    def move_vertical(self, command):
        for i in range(4):
            self.motor[i] = -0.5 if command == 'up' else 0.5

    def move_horizontal(self, command):
        self.motor[4] = 0.5 if command == 'forward' or command == 'right' else -0.5
        self.motor[5] = 0.5 if command == 'forward' or command == 'left' else -0.5

    def talker(self):
        rospy.loginfo(self.motor)
        self.pub_motor.publish(Float64MultiArray(data = self.motor))

    
if __name__ == '__main__':
    controller = Controller()
