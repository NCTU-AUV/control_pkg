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
#from tf.transformations import euler_from_quaternion

#DEPTH_OFFSET = 10

class Depth_gauge:
    def __init__(self):
        #self.arduino_port = glob.glob('/dev/ttyACM*')[0]       
        
        #self.arduino = serial.Serial(self.arduino_port, 115200, timeout=1)
        self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        while not self.arduino.is_open:
            self.arddfsdfuino.open()
            print("fail to open the arduino")

        rospy.on_shutdown(self.shutdown)
        rospy.init_node('Bar30', anonymous=True)

        #For PID
        self.pub = rospy.Publisher('Depth', Float64, queue_size=10) #[roll, pitch, yaw]
        #self.depth_pub = rospy.Publisher('Depth', Float64, queue_size=10) 

        '''
        #For visualize
        self.imu_visualize_data = Imu()
        self.imu_visualize_data.header.frame_id = "map"
        self.imu_visualize_pub = rospy.Publisher('IMU/Visualize', Imu, queue_size=10)
        '''
        self.imu_t = threading.Thread(target=self.get_data)
        #self.imu_t = threading.Thread(target=self.get_data, daemon=True)
        self.imu_t.start()

        #self.rate = rospy.Rate(100)
        
        rospy.spin()
    
    def cal(self, x):
        a = 5.667
        b = -1127.97
        
        # y = ax+b
        return a*x+b

    def get_data(self):
        data = -1

        while self.arduino.is_open:
            try:
                raw_data = self.arduino.readline()
                #print('arduino raw data: ')
                #print(raw_data)
                #data = unpack('fffc', raw_data) 
                data = self.cal(float(raw_data))
                print(raw_data, data)

            except Exception as e:
                print('oops')
                print(e)
                # time.sleep(0.5)
                #print(attitude)
           
            #attitude = euler_from_quaternion(data[0:4])
            #attitude = [x * 180 / math.pi for x in attitude] 
            '''
            attitude = list(data[0:3])

            roll = attitude[0]
            if roll > 0:
                roll = roll - 180
            elif roll < 0:
                roll = roll + 180
            attitude[0] = roll

            #roll, pitch, yaw
            rospy.loginfo(attitude)
            self.arr_pub.publish(Float64MultiArray(data=attitude))
            '''
            #imu.orientation
            #self.imu_visualize_data.orientation.w = data[3]
            #self.imu_visualize_data.orientation.x = data[0]
            #self.imu_visualize_data.orientation.y = data[1]
            #self.imu_visualize_data.orientation.z = data[2]
            #self.imu_visualize_data.orientation_covariance[0] = -1.0

            #imu.angular_velocity
            #self.imu_visualize_data.angular_velocity.x = data[4]
            #self.imu_visualize_data.angular_velocity.y = data[5]
            #self.imu_visualize_data.angular_velocity.z = data[6]
            #self.imu_visualize_data.angular_velocity_covariance[0] = -1.0
            
            #imu.linear_acceleration
            #self.imu_visualize_data.linear_acceleration.x = data[7]
            #self.imu_visualize_data.linear_acceleration.y = data[8]
            #self.imu_visualize_data.linear_acceleration.z = data[9]
            #self.imu_visualize_data.linear_acceleration_covariance[0] = -1.0
                
            #self.imu_visualize_pub.publish(self.imu_visualize_data)

	    #imu.depth
            self.pub.publish(data)

            #data[10] = data[10] - DEPTH_OFFSET
            #print(data)
            #print(type(self.imu_visualize_data))

            #self.rate.sleep() #100 hz

    def shutdown(self):
        self.arduino.close()
        print('\nbye')

def main():
    Depth_gauge()

if __name__ == '__main__':
    main()
