#!/usr/bin/env python3

import time
import threading
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


class Test_Dive:
    def __init__(self) -> None:
        self.ac_force = [0 for _ in range(8)]
        self.coe_force = [-1 for _ in range(8)]
        self.pub = rospy.Publisher('Motors_Force', Float64MultiArray, queue_size=10)

        rospy.init_node('talker', anonymous=True)
        rospy.on_shutdown(self.stop_motor)

        self.t = threading.Thread(target=self.input_motor)
        
        self.run()
        # self.diving()

    def run(self):
        rate = rospy.Rate(10)  # 10hz
        
        self.dc_force = 2
        force = [self.dc_force for _ in range(8)]   
        
        force[0] += 0.00
        force[1] += 0.00
        force[2] += 0.00
        force[3] += 0.00
        force[4] = 0.01 #0.05
        force[5] = 0.04 #0.07
        force[6] = 0.4 # clock-wise

        self.coe_force = [-1, -1, -1, -1, 0, 0, 0, 0]

        for i in range(8):
            force[i] += self.ac_force[i]
            force[i] *= self.coe_force[i]
        
        while not rospy.is_shutdown():  
            rospy.loginfo(Float64MultiArray(data=force))
            self.pub.publish(Float64MultiArray(data=force))
            rate.sleep()

    def input_motor(self):
        print('Status:')
        print(f'DC_force: {self.dc_force}')
        print('AC_force:')
        print(self.ac_force)
        
        f1 = input('Which function:\n1 Change dc force\n2 Change specific motor force\n')
        if f1 == '1':
            new_dc_force = float(input('New DC force: '))
            self.dc_force = new_dc_force
        elif f1 == '2':
            motor, ac_force = input('Which motor and ac_force:\n').split()
            motor = int(motor)
            ac_force = float(ac_force)
            self.ac_force[motor] = ac_force

    def diving(self):
        # rate = rospy.Rate(10) # 10hz
        for i in range(50):
            force = [self.dc_force for _ in range(8)]
            self.pub.publish(Float64MultiArray(data=force))
            time.sleep(0.1)
        
        self.stop_motor()

    def stop_motor(self):
        force = [0 for _ in range(8)]
        for i in range(3):
            print(force)
            self.pub.publish(Float64MultiArray(data=force))
            time.sleep(0.1)
        print('Finishing stopping motors')


if __name__ == '__main__':
    Test_Dive()
