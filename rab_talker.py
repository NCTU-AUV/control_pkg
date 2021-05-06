#! /usr/bin/python3

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
import traceback as tb
from rabboni import Rabboni
import math
import time

rab = Rabboni()
rab.connect()
rab.set_sensor_config(8, 500, 20, 100)
position_lr = 0
position_fb = 0
timer = time.time()

preW_fb = 0
preW_lr = 0


rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('command',String, queue_size=10)
pub2 = rospy.Publisher('value',Float32,queue_size=10)
#rate = rospy.Rate(10) # 10hz

def usb_custom_callback(status):
    global position_lr
    global position_fb
    global timer
    global preW_fb
    global preW_lr
    check = 1
    a = 0.7

    if(abs(preW_fb-status['Gyr'][0]) > 0):
        position_fb = (1 - a) * (position_fb + (status['Gyr'][0] * math.pi/180) * (time.time() - timer)) + a * status['Acc'][1]
    if(abs(preW_lr-status['Gyr'][1]) > 0):
        position_lr = (1 - a) * (position_lr + (status['Gyr'][1] * math.pi/180) * (time.time() - timer)) + a * status['Acc'][0]
    
    #Front and Back
    if status['Acc'][1] < -0.1 and (-0.3 > position_fb):
        s = 'front'
        check = 0
    elif status['Acc'][1] > 0.1 and (0.3 < position_fb): 
        s = 'back'
        check = 0
    
    #Left and Right
    elif status['Acc'][0] < -0.1 and (-0.2 > position_lr) and check: 
        s = 'right'
    elif status['Acc'][0] > 0.1 and (0.2 < position_lr) and check:
        s = 'left'
    else:
        s = 'stable'

    print(s + ' ' + str(position_fb))
    timer = time.time()
    preW_fb = status['Gyr'][0]
    preW_lr = status['Gyr'][1]
    pub.publish(s)
    pub2.publish(position_fb)




def talker():
	
	rab.start_fetching_status(custom_callback=usb_custom_callback)
	rab.polling_status()
	
		

if __name__ == '__main__':
	try:
		talker()
	except AssertionError:
		print("Bye")
		rab.disconnect()
	except KeyboardInterrupt:
		print("Bye")
		rab.disconnect()
	except rospy.ROSInterruptException:
		pass
		rab.disconnect()
	finally:
	    rab.disconnect()
		

