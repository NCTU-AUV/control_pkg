#! /usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64MultiArray, Int16
import math
import time
#from control_pkg.srv import PidControl, PidControlResponse

class Controller():
    def __init__(self, command = 'stop'):
        rospy.init_node('Controller', anonymous=True)
        self.pub_motor = rospy.Publisher('Motors_Force_Command', Float64MultiArray, queue_size=10)
        self.pub_on_attitude = rospy.Publisher('On/Attitude', Int16, queue_size=10)
        self.pub_on_depth = rospy.Publisher('On/Depth', Int16, queue_size=10)

        self.command_joy = command
        self.command_CV = command
        self.motor = [0.0]*6
        
        print('ready to controll vehicle')

        rospy.Subscriber('cmd', String, self.callback_joystick)
        rospy.Subscriber('tracking_command', String, self.callback_CV)
        rospy.spin()

    def callback_joystick(self, data):
        self.command_joy = data.data
        print('command from joystick ' + data.data)
        self.switch(data.data)
        self.talker()

    def callback_CV(self, data):
        self.command_CV = data.data
        print('command from CV ' + data.data)
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
        self.pub_on_attitude.publish(data = 1)
        self.pub_on_depth.publish(data = 1)
    
    def move_vertical(self, command):
        for i in range(4):
            self.motor[i] = -1.92 if command == 'up' else 0.28
        self.pub_on_attitude.publish(data = 1)    
        self.pub_on_depth.publish(data = 0)

    def move_horizontal(self, command):
        if command == 'forward':
            self.motor[4] = 1.5
            self.motor[5] = 1.0
        elif command == 'backward':
            self.motor[4] = -1.0
            self.motor[5] = -1.5
        elif command == 'right':
            self.motor[4] = 0.8
            self.motor[5] = -0.8
        else:
            self.motor[4] = -0.4
            self.motor[5] = 0.4

        self.pub_on_attitude.publish(data = 0)
        self.pub_on_depth.publish(data = 1)

    def talker(self):
        rospy.loginfo(self.motor)
        self.pub_motor.publish(Float64MultiArray(data = self.motor))
    
if __name__ == '__main__':
    controller = Controller()
