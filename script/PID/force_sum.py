#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import traceback
import numpy as np
import math
import time

class Sum():
	def __init__(self):
		self.state = 0
		self.bdata = np.array([0,0,0,0,0,0,0,0])
		self.ddata = np.array([0,0,0,0,0,0,0,0])
		self.fdata = np.array([0,0,0,0,0,0,0,0])
		self.tdata = np.array([0,0,0,0,0,0,0,0])
		rospy.init_node('total_force', anonymous=True)
		self.force_pub = rospy.Publisher('/force/total',Float32MultiArray,queue_size=10) 
		rospy.Subscriber("/force/balance", Float32MultiArray, self.balance_update,queue_size=1)
		rospy.Subscriber('/force/depth', Float32MultiArray, self.depth_update,queue_size=1)
		rospy.Subscriber('/force/forward', Float32MultiArray, self.forward_update,queue_size=1)
		rospy.Subscriber('/AUVmanage/state',Int32,self.state_change)
	def Main(self):
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			if self.state == 1 or self.state == 3:
				self.tdata = self.bdata+self.ddata+self.fdata
				force_data = Float32MultiArray(data = self.tdata)
				self.force_pub.publish(force_data)
			elif self.state == 0 or self.state == 2:
				self.tdata = [0,0,0,0,0,0,0,0]
				force_data = Float32MultiArray(data = self.tdata)
				self.force_pub.publish(force_data)
			rate.sleep()
	def balance_update(self,data):
		self.bdata = np.array(data.data)
	def depth_update(self,data):
		self.ddata = np.array(data.data)
	def forward_update(self,data):
		self.fdata = np.array(data.data)
	def state_change(self,data):
		self.state = data.data
if __name__ == "__main__":
	try:
		po = Sum()
		po.Main()
	except Exception as e:
	    exstr = traceback.format_exc()
	    print(exstr)
