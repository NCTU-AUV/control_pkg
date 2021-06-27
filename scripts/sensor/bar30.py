#! /usr/bin/env python

import serial
import glob
import rospy
import threading
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Imu
import time
import random
import math
from struct import unpack

class IMUAttitude:
    def __init__(self):
        #self.arduino_port = glob.glob('/dev/ttyACM*')[0]        
        #self.arduino = serial.Serial(self.arduino_port, 115200, timeout=1)
        self.arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

        while not self.arduino.is_open:
            self.arddfsdfuino.open()
            print("fail to open the arduino")

        rospy.on_shutdown(self.shutdown)
        rospy.init_node('Depth', anonymous=True)

        self.pub = rospy.Publisher('Depth', Float64, queue_size=10)
        self.imu_t = threading.Thread(target=self.get_data)
        self.imu_t.start()
        
        rospy.spin()
   
    def get_data(self):
        data = -1

        while self.arduino.is_open:
            try:
                raw_data = self.arduino.readline()
                # print('type', type(raw_data), raw_data)
                data = float(raw_data)
                print(raw_data, data)

            except Exception as e:
                print('oops')
                print(e)
                # time.sleep(0.5)
                #print(attitude)
           
	        #imu.depth
            self.pub.publish(data) 

    def shutdown(self):
        self.arduino.close()
        print('\nbye')

def main():
    IMUAttitude()

if __name__ == '__main__':
    main()
