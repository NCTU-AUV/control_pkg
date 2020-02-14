#!/usr/bin/env python
import AUV_physics
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
import time
import traceback
import rosparam
import math


class Forward():
	def __init__(self):
		rospy.init_node('forward_PID', anonymous=True)
		self.state = 0 
		self.Po = AUV_physics.AUV()
		self.yaw_error = 0
        self.yaw_error_I = 0
        self.yaw_error_I_array = [0,0,0,0,0,0,0,0,0,0]
		self.forward_pub = rospy.Publisher('/force/forward',Float32MultiArray,queue_size=1)
		self.vel = 0.1
		rospy.Subscriber('/error/yaw', Float32, self.yaw)
		rospy.Subscriber('/AUVmanage/state',Int32,self.state_change)
		rospy.Subscriber('/depth', Float32, self.depth_cb)
	def Main(self):
		r = rospiy.Rate(20)
		tStart = time.time()
		while not rospy.is_shutdown():
			if time.time() - tStart >0.5:
				try:
					para = rosparam.get_param('/PIDpara/yaw')
					self.tune_yaw = para[0]
                    Kp = para[1]
                    Ki = para[2]
                    Kd = para[3]
                    tStart = time.time()
				except Exception as e:
					exstr = traceback.format_exc()
					print(exstr)
			if self.state == 1 or self.state == 2  and self.depth >0.6 and self.depth <0.75:
				drag_force = self.Po.drag_effect(np.array([self.vel,0,0,0,0,0])+np.array(0,0,0,0,0,self.tune_yaw))
			    self.yaw_error_I = self.yaw_error + self.yaw_error_I - self.yaw_error_I_array[0]
                self.yaw_error_I_array[0] = self.yaw_error
                self.yaw_error_I_array = np.roll(self.yaw_error_I_array,1)
                forward_force = drag_force + np.array([0,0,0,0,0,self.yaw_error*Kp+self.yaw_error_I*Ki])
				#print forward_force
				forward_force = np.dot(self.Po.Trust_inv,forward_force)
				#print(forward_force)
				#print self.depth
				force_data = Float32MultiArray(data = forward_force)
				self.forward_pub.publish(force_data)
			elif self.state ==3:
				drag_force = self.Po.drag_effect(np.array([self.vel,0,0,0,0,0]) +np.array(0,0,0,0,0,self.tune_yaw))
                self.yaw_error_I = self.yaw_error + self.yaw_error_I - self.yaw_error_I_array[0]
                self.yaw_error_I_array[0] = self.yaw_error
                self.yaw_error_I_array = np.roll(self.yaw_error_I_array,1)
                forward_force = drag_force + np.array([0,0,0,0,0,self.yaw_error*Kp+self.yaw_error_I*Ki])
				forward_force = np.dot(self.Po.Trust_inv,forward_force)
				force_data = Float32MultiArray(data = forward_force)
				self.forward_pub.publish(force_data)
			elif self.state ==1 or self.state == 2:
				self.forward_pub.publish(Float32MultiArray(data = [0,0,0,0,0,0,0,0]))

			elif self.state ==0:
				self.forward_pub.publish(Float32MultiArray(data = [0,0,0,0,0,0,0,0]))
	
			r.sleep()


	def yaw(self,data):
		self.yaw_error = data.data
	def state_change(self,data):
		self.state = data.data
	def depth_cb(self,data):
		self.depth = data.data
if __name__ == "__main__":
	try:
		Po = Forward()
		Po.Main()
	except Exception as e:
		exstr = traceback.format_exc()
		print(exstr)
