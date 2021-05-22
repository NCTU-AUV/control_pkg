#!/usr/bin/env python3

# ------------------------------
# Joystick Command Emitter
# 
# Ros joystick tutorial:
# http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
# 
# Setup in summary:
# $ ls -l /dev/input/js0
# $ sudo chmod a+rw /dev/input/js0
# $ rosrun joy joy_node 
# ------------------------------

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy

class Joystick:
    def __init__(self):
        self.K = 4      # to enlarge the result
        self.ly = 0
        self.rx = 0
        self.ry = 0
        
        rospy.init_node('joy_cmd_emitter', anonymous=True ) # node_name
        rospy.Subscriber("joy", Joy, self.callback)
        
        self.pub = rospy.Publisher('cmd', String, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.cmd_pub()
        

    def callback(self, data):             
        self.ly = data.axes[1] * self.K     # {0, -1, 1}
        self.rx = data.axes[2] * self.K     # [-1, 1]
        self.ry = data.axes[3] * self.K     # [-1, 1]
        # print(data)
    
    
    def cmd_pub(self):       
        while not rospy.is_shutdown():
            if self.ry > 0.5:
                cmd_str = 'forward'
            elif self.ry < -0.5:
                cmd_str = 'backward'
            elif self.rx > 0.5:
                cmd_str = 'left'
            elif self.rx < -0.5:
                cmd_str = 'right'
            elif self.ly > 0.5:
                cmd_str = 'up'
            elif self.ly < -0.5:
                cmd_str = 'down'
            else:
                cmd_str = 'stop'
            self.pub.publish(cmd_str)
            print('cmd_str :', cmd_str)
            
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Joystick()
    except rospy.ROSInterruptException:
        print('Bye')
