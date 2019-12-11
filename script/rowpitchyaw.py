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
        rospy.init_node('IMU_update', anonymous=True)
        self.pose_pub = rospy.Publisher('altitude', PoseStamped, queue_size=1)
        self.Row_pub = rospy.Publisher('Row', Float32, queue_size=1)
        self.Pitch_pub = rospy.Publisher('Pitch', Float32, queue_size=1)
        self.Yaw_pub = rospy.Publisher('Yaw', Float32, queue_size=1)
        self.Eular_pub = rospy.Publisher('Eular', Float32MultiArray , queue_size=1)
        self.row =0.
        self.pitch =0.
        self.yaw =0.
        self.Eular = [0.,0.,0.]
    def get_arduino(self):
        serial_port = popen('readlink -f /dev/serial/by-path/pci-0000\:00\:14.0-usb-0\:1\:1.0').read().split()[0]
        s = serial.Serial(serial_port,baudrate=115200)
        while not rospy.is_shutdown():
            if s.in_waiting:
                data = s.read(80)
                data =data.replace('\x00','')
                data =data.split()
                self.row = float(data[1])
                self.pitch = float(data[3])
                self.yaw = float(data[5])
                self.Eular = [self.row ,self.pitch,self.yaw]
                qua = tf.transformations.quaternion_from_euler(self.row*np.pi/180, self.pitch*np.pi/180, 0)
                altitude = Quaternion(x=qua[0],y=qua[1],z=qua[2],w=qua[3])
                altitude = Pose(orientation=altitude)
                head = Header(frame_id = 'my_frame')
                altitude = PoseStamped(pose = altitude,header =head)

                self.pose_pub.publish(altitude)
    def start(self):
        t = threading.Thread(target=self.get_arduino)
        t.start()
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.Row_pub.publish(self.row)
            self.Pitch_pub.publish(self.pitch)
            self.Yaw_pub.publish(self.yaw)
            self.Eular_pub.publish(Float32MultiArray(data = self.Eular))
            r.sleep()
if __name__ == "__main__":
    try:
        GD=GET_DATA()
        GD.start()
    except rospy.ROSInterruptException:
        pass
