#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import traceback
import numpy as np
import math
import time

class Main():
	def __init__(self):
		print 1
		
		self.bdata = np.array([0,0,0,0,0,0,0,0])
		self.ddata = np.array([0,0,0,0,0,0,0,0])
		self.fdata = np.array([0,0,0,0,0,0,0,0])
		self.tdata = np.array([0,0,0,0,0,0,0,0])
		rospy.init_node('total_force', anonymous=True)
		rate = rospy.Rate(50)
		self.force_pub = rospy.Publisher('/force/total',Float32MultiArray,queue_size=10) 
		rospy.Subscriber("/force/balance", Float32MultiArray, self.balance_update,queue_size=1)
		rospy.Subscriber('/force/depth', Float32MultiArray, self.depth_update,queue_size=1)
		rospy.Subscriber('/force/forward', Float32MultiArray, self.forward_update,queue_size=1)
		while not rospy.is_shutdown():
			self.tdata = self.bdata+self.ddata+self.fdata
			force_data = Float32MultiArray(data = self.tdata)
			self.force_pub.publish(force_data)
			rate.sleep()
	def balance_update(self,data):
		self.bdata = np.array(data.data)
	def depth_update(self,data):
		self.ddata = np.array(data.data)
	def forward_update(self,data):
		self.fdata = np.array(data.data)
if __name__ == "__main__":
	try:
		Main()
	except Exception as e:
	    exstr = traceback.format_exc()
	    print(exstr)
