#! /usr/bin/env python3

import serial
import glob
import rospy
import threading
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import time
import random

class IMUAttitude:
    def __init__(self):
        self.arduino_port = glob.glob('/dev/ttyACM*')[0]       
        
        self.arduino = serial.Serial(self.arduino_port, 9600, timeout=1)
        if not self.arduino.is_open:
            self.arduino.open()

        rospy.on_shutdown(self.shutdown)
        rospy.init_node('IMU_9250', anonymous=True)
        self.arr_pub = rospy.Publisher('IMU/Attitude', Float64MultiArray, queue_size=10)
        
        #new
        self.roll_pub = rospy.Publisher('IMU/Roll', Float64, queue_size=10)
        self.pitch_pub = rospy.Publisher('IMU/Pitch', Float64, queue_size=10)
        self.yaw_pub = rospy.Publisher('IMU/Yaw', Float64, queue_size=10)
        self.temp_pub = rospy.Publisher('IMU/Temperature', Float64, queue_size=10)
        
        self.imu_t = threading.Thread(target=self.get_data, daemon=True)
        self.imu_t.start()

        rospy.spin()
    
    def get_data(self):
        while self.arduino.is_open:
            try:
                raw_data = self.arduino.readline()
                #print('arduino raw data: ')
                #print(raw_data)
                attitude = [float(val) for val in raw_data.split()]
                if len(attitude) != 4:
                    continue
                roll = float(attitude[0])
                pitch = float(attitude[1])
                yaw = float(attitude[2])
                temperature = float(attitude[3])

            except Exception as e:
                print('oops')
                print(e)
                # time.sleep(0.5)
                #print(attitude)
            
            #rospy.loginfo(attitude)
            self.arr_pub.publish(Float64MultiArray(data=attitude))
            self.roll_pub.publish(Float64(data=roll))
            self.pitch_pub.publish(Float64(data=pitch))
            self.yaw_pub.publish(Float64(data=yaw))
            self.temp_pub.publish(Float64(data=temperature))

    def shutdown(self):
        self.arduino.close()
        print('\nbye')

def main():
    IMUAttitude()

if __name__ == '__main__':
    main()
