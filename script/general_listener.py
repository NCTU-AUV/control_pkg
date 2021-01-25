#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def listener(number):
    if number == "1":
        rospy.Subscriber('IMU/Attitude', Float64MultiArray, callback)
    elif number == "2":
        rospy.Subscriber('Motors_Force', Float64MultiArray, callback)
    
    rospy.spin()

def callback(data):
    if number == "1":
        for i in range(3):
            print(data.data[i])
    elif number == "2":
        for i in range(8):
            print(data.data[i])

if __name__ == '__main__':    
    rospy.init_node('general_listener', anonymous=True)

    print("which topic do you want to listen : ")
    print("1 for IMU/Attitude")
    print("2 for Motors_Force")
    number = input()
    print("now listening %s", number)
    
    listener(number)