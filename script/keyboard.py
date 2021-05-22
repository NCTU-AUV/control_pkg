#!/usr/bin/env python3
import rospy
import sys
import tty, termios
from std_msgs.msg import String

def talker():
        rospy.init_node('keyboard', anonymous=True)
        pub=rospy.Publisher('cmd', String, queue_size=10)
        rate=rospy.Rate(1)
        print ("reading from keyboard")
        print ("w:forward a:left s:back d:right c:stop r:up f:down")
        while not rospy.is_shutdown():
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                #old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
                try:
                        tty.setraw (fd)
                        ch = sys.stdin.read(1)
                finally:
                        #pass
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                str1="stop"
                if ch == 'w':
                        str1 = "forward"
                elif ch == 'a':
                        str1 = "left"
                elif ch == 's':
                        str1 = "backward"
                elif ch == 'd':
                        str1 = "right"
                elif ch == 'r':
                        str1 = "up"
                elif ch == 'f':
                        str1 = "down"
                elif ch == 'c':
                        str1 = "stop"
                elif ch == 'q':
                        exit()
                rospy.loginfo(str1)
                pub.publish(str1)


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print('hit')
