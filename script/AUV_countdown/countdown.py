#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32


class Main():
	def __init__(self):
		rospy.init_node('countdown', anonymous=True)
		rospy.Subscriber('/AUVmanage/countdowner', Int32, self.countdown_and_pub)
		self.start = rospy.Publisher('/AUVmanage/state',Int32,queue_size=1)
		while not rospy.is_shutdown():
			pass
	def countdown_and_pub(self,countdowner):
		print(type(countdowner.data))
		time.sleep(countdowner.data)
		rospy.loginfo("start")
		self.num = Int32(data = 1)
		self.start.publish(self.num)


	

if __name__ == "__main__":
	try:
		Main()
	except Exception as e:
		exstr = traceback.format_exc()
		print(exstr)