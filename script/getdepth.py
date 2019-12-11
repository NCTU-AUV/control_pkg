#!/usr/bin/env python

# from http://forum.arduino.cc/index.php?topic=137635.msg1270996#msg1270996
import traceback
import time, threading, sys
import serial
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import tf.transformations
from geometry_msgs.msg import PoseStamped,Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from os import popen

        # convert data to 16bit int numpy array
        #data = np.fromstring(data, dtype=np.uint8)
        #print(last)
class GET_DATA():
    def __init__(self):
        rospy.init_node('depth_update', anonymous=True)
        self.depth_pub = rospy.Publisher('depth', Float32, queue_size=1)
        self.depth = 1
    def get_arduino(self):
        serial_port = popen('readlink -f /dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0').read().split()[0]
        s = serial.Serial(serial_port,baudrate=9600)
        while not rospy.is_shutdown():
            if s.in_waiting:
                data = s.readline()
                data =data.replace('\x00','')
                data =data.split()
                if data[0] == 'Depth:':
                    self.depth = float(data[1])
                    depth = Float32(data = 1)
                    #print(self.depth)
    def start(self):
        t = threading.Thread(target=self.get_arduino)
        t.start()
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.depth_pub.publish(self.depth)
            r.sleep()
if __name__ == "__main__":
    try:
        GD=GET_DATA()
        GD.start()
    except rospy.ROSInterruptException:
        pass