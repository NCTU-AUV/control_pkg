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
'''
state_data = 0
depth_data = 0
depthL = [0 for i in range(10)]
depthR = [0.3, 0.35, 0.6]

Fkp = 0
FincreaseRate = 0.05

def depth_cb(data):
    global depthL, Fkp, FincreaseRate, depth_data, state_data
    depth_data=data.data
    for i in range(len(depthL)-1, 0, -1):
        depthL[i] = depthL[i-1]
    depthL[0] = depth_data-depthR[1]
    if (state_data%2) == 0 or state_data == -1:
        pub_data = [0 for i in range(8)]
        pub_data = Float32MultiArray(data = pub_data)
        pub1.publish(pub_data)
        return
    
    D = depthL[0] - depthL[2]
    E = depth_data-depthR[1]
    Fkp = Fkp+FincreaseRate*E+FincreaseRate*D*1


    if depth_data < depthR[0]:
        Fkp = Fkp-FincreaseRate*5
    elif depth_data > depthR[2]:
        Fkp = Fkp+FincreaseRate*5
    elif depth_data > depthR[1]:
        Fkp = Fkp+FincreaseRate
    else:
        if depthL[2] < depthL[0]:
            Fkp = Fkp+FincreaseRate*3
        elif depthL[2] > depthL[0]:
            Fkp = Fkp-FincreaseRate*10

    #test_mat_all = np.matrix([[0], [0], [az], [0], [0], [0]])
    result_F = Taz*Fkp
    pub_data = [result_F[i] for i in range(8)]
    pub_data = Float32MultiArray(data = pub_data)
    pub1.publish(pub_data)

def Kp_cb(data):
    global FincreaseRate
    FincreaseRate = data.data
    
def state_cb(data):
    global state_data, Fkp, init_K
    state_data = data.data
    if (state_data%2) == 0:
        Fkp = 0
    if (state_data%2) == 1:
        Fkp = init_K
'''

class Main():
    def __init__(self):
        rospy.init_node('depth_PID', anonymous=True)
        self.state = 0 
        rospy.Subscriber('/AUVmanage/state',Int32,self.state_change)
        self.Po = AUV_physics.AUV()
        self.depth_target = 1.
        rospy.Subscriber('/depth', Float32, self.depth_cb)
        self.depth_pub = rospy.Publisher('/force/depth',Float32MultiArray,queue_size=10)
        tStart = time.time()
        self.depth_error_I =0.
        self.last_error = 0.
        self.depth_PID = [0,0,0]
        tStart = time.time()
        while not rospy.is_shutdown():
        	if time.time() - tStart >0.5:
	            try:
	                self.depth_PID = rosparam.get_param('/PIDpara/depth')
	                tStart = time.time()
	            except Exception as e:
	                exstr = traceback.format_exc()
	                print(exstr)

    def depth_cb(self,data):
        if self.state == 1: #normal state
            tStart = time.time()
            depth_data=data.data
            Kp = self.depth_PID[0]
            Ki = self.depth_PID[1]
            Kd = self.depth_PID[2]
            depth_error = self.depth_target-depth_data
            rospy.loginfo('depth error is :' + str(depth_error))
            depth_force = [0,0,0,0,0,0]
            if depth_error > 0.4:
                depth_force[2] = 30
            elif depth_error < -0.4:
                depth_force[2] = -30
            else:
                self.depth_error_I = self.depth_error_I*0.8+depth_error
                depth_error_D = depth_error - self.last_error
                self.last_error = depth_error
                depth_force[2] = Kp*depth_error +  Ki*self.depth_error_I + Kd*depth_error_D
            a = self.Po.buoyancy_effect()
            depth_force[2] = depth_force[2] - a[2][0]
            depth_force = np.dot(self.Po.Trust_inv,depth_force)
            print(depth_force)
            force_data = Float32MultiArray(data = depth_force)
            self.depth_pub.publish(force_data)
            print(time.time()-tStart)
        if self.state == 0:
            self.depth_pub.publish(Float32MultiArray(data = [0,0,0,0,0,0,0,0]))
    def Kp_cb():
        pass
    def state_change(self,data):
        self.state = data.data

if __name__ == "__main__":
    try:
        Main()
    except Exception as e:
        exstr = traceback.format_exc()
        print(exstr)