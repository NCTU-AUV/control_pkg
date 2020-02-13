#!/usr/bin/env python
# license removed for brevity

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

class Depth():
    def __init__(self):
        rospy.init_node('depth_PID', anonymous=True)
        self.state = 0 
        self.Po = AUV_physics.AUV()
        self.depth_target = 0.7
        # for PID control
        self.depth_error_I =0.
        self.last_error = 0.
        self.depth_PID = [0,0,0]
        self.depth_pub = rospy.Publisher('/force/depth',Float32MultiArray,queue_size=1)
        self.depth_data=0
        rospy.Subscriber('/AUVmanage/state',Int32,self.state_change)
        rospy.Subscriber('/Eular', Float32MultiArray, self.Eular_updata)
        rospy.Subscriber('/depth', Float32, self.depth_cb)
    def Main(self):
        r = rospy.Rate(50)
        tStart = time.time()
        while not rospy.is_shutdown():
            if time.time() - tStart >0.5:
                try:
                    self.depth_PID = rosparam.get_param('/PIDpara/depth')
                    tStart = time.time()
                except Exception as e:
                    exstr = traceback.format_exc()
                    print(exstr)
            if self.state == 1 or self.state == 2: #normal state
                Kp = self.depth_PID[0]
                Ki = self.depth_PID[1]
                Kd = self.depth_PID[2]
                depth_error = self.depth_target-self.depth_data
                #rospy.loginfo('depth error is :' + str(depth_error))
                depth_force = [0,0,0,0,0,0]
                if depth_error > 04.:
                    depth_force[2] = Kp*0.4
                elif depth_error < -0.4:
                    depth_force[2] = Kp*-0.4
                else:
                    # for I control
                    self.depth_error_I = self.depth_error_I+depth_error
                    # for D control
                    depth_error_D = depth_error - self.last_error
                    self.last_error = depth_error
                    depth_force[2] = Kp*depth_error +  Ki*self.depth_error_I + Kd*depth_error_D
                a = self.Po.buoyancy_effect()
                depth_force[2] = depth_force[2] - a[2][0]
                depth_force = np.dot(self.Po.Trust_inv,depth_force)
                #print(depth_force)
                force_data = Float32MultiArray(data = depth_force)
                self.depth_pub.publish(force_data)
                #print(time.time()-tStart)
            if self.state == 0:
                self.depth_pub.publish(Float32MultiArray(data = [0,0,0,0,0,0,0,0]))
            r.sleep()
    def Eular_updata(self,data):
        data = data.data
        self.Po.Eular_update(data[0],data[1],data[2])
        #print data
    def depth_cb(self,data):
        self.depth_data=data.data
    def state_change(self,data):
        self.state = data.data

if __name__ == "__main__":
    try:
        po =Depth()
        po.Main()
    except Exception as e:
        exstr = traceback.format_exc()
        print(exstr)
