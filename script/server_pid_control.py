#!/usr/bin/env python3

from control_pkg.srv import PidControl, PidControlResponse
import rospy

def handle_pid_control(req):
    print("Get control msg [%f %f %f %f %f %f %f %f %f %f %f %f ]"%
        (req.p1, req.po1, req.i1, req.io1, req.d1, req.do1, req.p2, req.po2, req.i2, req.io2, req.d2, req.do2))
    return PidControlResponse(True)

def pid_control_server():
    rospy.init_node('pid_control_server')
    s = rospy.Service('pid_control', PidControl, handle_pid_control)
    print("Ready to get control msg.")
    rospy.spin()

if __name__ == "__main__":
    pid_control_server()