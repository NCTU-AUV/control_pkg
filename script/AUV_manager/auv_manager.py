#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import rosparam
import yaml
import traceback
class Manager():
	def __init__(self):
		rospy.init_node('countdown', anonymous=True)
		self.param_list = []
		self.param_list.append({'name':'/PIDpara/altitude','file':r'/home/nctu-auv/catkin_ws/src/auv_control/config/altitude.yaml'})
		self.param_list.append({'name':'/PIDpara/depth','file':r'/home/nctu-auv/catkin_ws/src/auv_control/config/depth.yaml'})
		self.param_list.append({'name':'/PIDpara/yaw','file':r'/home/nctu-auv/catkin_ws/src/auv_control/config/yaw_tune.yaml'})
		print(self.param_list[1]['name'])
	def start(self):
		rospy.Subscriber('/AUVmanage/countdowner', Int32MultiArray, self.countdown)
		rospy.Subscriber('/AUVmanage/dumpparam', Int32, self.dump_param)
		self.start = rospy.Publisher('/AUVmanage/state',Int32,queue_size=1)
		while not rospy.is_shutdown():
			pass
	def countdown(self,countdowner):
		print(type(countdowner.data))
		time.sleep(countdowner.data[1])
		rospy.loginfo("start")
		self.num = Int32(data = countdowner.data[0])
		self.start.publish(self.num)
	def dump_param(self, param_num):
		param_num = param_num.data
		data = rospy.get_param(self.param_list[param_num]['name'])
		dict_data = {self.param_list[param_num]['name']:data}
		print(dict_data)
		with open(self.param_list[param_num]['file'],'w+') as f:
			print(yaml.dump(dict_data,f, default_flow_style = False))
if __name__ == "__main__":
	try:
		mang=Manager()
		mang.start()
	except Exception as e:
		exstr = traceback.format_exc()
		print(exstr)
