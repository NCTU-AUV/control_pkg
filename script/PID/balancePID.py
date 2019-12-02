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


class Main():
	def __init__(self):
		rospy.init_node('balance_PID', anonymous=True)
		self.state = 0
		rospy.Subscriber('/Eular', Float32MultiArray, self.balance_cb)
		self.Po = AUV_physics.AUV()
		self.balance_pub = rospy.Publisher('/force/balance',Float32MultiArray,queue_size=1)
		rospy.Subscriber('/AUVmanage/state',Int32,self.state_change)
		self.balance_error_I =0.
		self.last_error = 0.
		self.balance_PID = [0,0,0]
		tStart = time.time()
		while not rospy.is_shutdown():

			if time.time() - tStart >0.5:
				try:
				    self.balance_PID = rosparam.get_param('/PIDpara/altitude')
				    self.tune_yaw = rosparam.get_param('/tune/yaw')
				    tStart = time.time()
				except Exception as e:
				    exstr = traceback.format_exc()
				    print(exstr)

	def balance_cb(self,data):
		if self.state == 1:
			Kp = self.balance_PID[0]
			Ki = self.balance_PID[1]
			Kd = self.balance_PID[2]
			theta = list(data.data)
			x= [0,0,0]
			x[0]=data.data[0]*math.pi/180.
			x[1]=data.data[1]*math.pi/180.
					#self.x[2]=data.data[2]*math.pi/180.
			x[2]=0
			R = AUV_physics.eulerAnglesToRotationMatrix(x)
			z_axis=np.dot(R,np.array([[0],[0],[1]])).T
			x=np.array([0,0,1])
			y=np.array([0,1,0])
			balance_error = AUV_physics.AUV().mass_effect(np.append([0,0,0],-np.cross(z_axis[0],x)))
			self.balance_error_I = self.balance_error_I*0.999+balance_error
			balance_error_D = balance_error - self.last_error
			self.last_error = balance_error
			balance_force = Kp*balance_error +  Ki*self.balance_error_I + Kd*balance_error_D
			#yaw_tune
			balance_force = balance_force + np.array([0,0,0,0,0,self.tune_yaw])
			balance_force = np.dot(self.Po.Trust_inv,balance_force)
			#print(balance_force)
			force_data = Float32MultiArray(data = balance_force)
			print force_data
			self.balance_pub.publish(force_data)
		elif self.state == 0:
			self.balance_pub.publish(Float32MultiArray(data = [0,0,0,0,0,0]))
	def state_change(self,data):
		self.state = data.data
if __name__ == "__main__":
	try:
		Main()
	except Exception as e:
		exstr = traceback.format_exc()
		print(exstr)