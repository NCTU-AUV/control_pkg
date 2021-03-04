#! /usr/bin/env python3

'''
The listener controlling motors with pca9685
I/P: array of 8 motor forces (kgf)
O/P: controll motors

PCA9685 I2C
http://blog.ittraining.com.tw/2015/07/raspberry-pi2-python-i2c-16-pwm-pca9685.html

Datasheet of T200
https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
'''

from smbus2 import SMBus
import time
import sys
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Float32


class MotorController:
    pca9685_addr = 0x40
    bus = SMBus(1)
    FORCE = (
        -4.069476226666667, -4.05057656, -4.023361039999999, -3.96439408,
        -3.9008912, -3.8691397599999995, -3.82378056, -3.8011009600000003,
        -3.75120584, -3.7149184799999997, -3.6559515200000003, -3.58791272,
        -3.5198739199999998, -3.45183512, -3.3974040800000003, -3.3112216,
        -3.25225464, -3.20235952, -3.1071052, -3.0390664, -2.9937072,
        -2.9392761600000004, -2.8621655199999996, -2.82134224, -2.77144712,
        -2.70794424, -2.6580491200000003, -2.5809384800000004, -2.54918704,
        -2.50836376, -2.45393272, -2.381358, -2.35414248, -2.27703184,
        -2.2362085599999997, -2.19538528, -2.11827464, -2.08198728,
        -2.02302032, -1.97766112, -1.94137376, -1.8597271999999998, -1.814368,
        -1.76447288, -1.7009699999999999, -1.66921856, -1.6102516,
        -1.5558205600000001, -1.51499728, -1.4877817599999998,
        -1.4378866399999999, -1.39706336, -1.34716824, -1.30180904,
        -1.25644984, -1.2020188, -1.16119552, -1.12037224, -1.1022285600000001,
        -1.05233344, -1.020582, -0.9797587200000001, -0.94347136, -0.90264808,
        -0.87089664, -0.82100152, -0.7847141599999999, -0.7438908799999999,
        -0.71667536, -0.680388, -0.6486365599999999, -0.6168851200000001,
        -0.58059776, -0.53977448, -0.5125589599999999, -0.48080752000000004,
        -0.44452016, -0.41730464, -0.3855532, -0.35380176, -0.32205032,
        -0.2948348, -0.25854743999999996, -0.23586784, -0.20865232, -0.1814368,
        -0.14968536000000002, -0.12700576000000002, -0.10432616, -0.08618248,
        -0.0680388, -0.04989512, -0.03628736, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.040823279999999997, 0.05443104000000001,
        0.07711064000000001, 0.09979024, 0.12700576, 0.15422128,
        0.18143679999999998, 0.21772416, 0.24947559999999996, 0.29029888,
        0.32205032, 0.35833768, 0.39916096, 0.43544832, 0.47173568,
        0.5125589599999999, 0.5579181599999999, 0.60327736, 0.63956472,
        0.680388, 0.72121128, 0.77564232, 0.8164656, 0.87089664, 0.907184,
        0.9480072799999999, 0.9933664799999999, 1.0341897599999998, 1.09769264,
        1.14305184, 1.18387512, 1.23830616, 1.27912944, 1.3290245600000001,
        1.38799152, 1.4378866399999999, 1.48324584, 1.53767688, 1.587572,
        1.6510748800000001, 1.69189816, 1.7599369599999999, 1.8234398400000003,
        1.8778708800000001, 1.9323019200000002, 1.9912688800000002, 2.04569992,
        2.1228105600000005, 2.1772416000000003, 2.21806488, 2.2770318400000003,
        2.381358, 2.43125312, 2.5174356, 2.5764025600000005,
        2.6535132000000003, 2.7306238400000002, 2.75783936, 2.8394859200000004,
        2.88938104, 2.97556352, 3.05267416, 3.1071052000000003, 3.17060808,
        3.21596728, 3.3021497600000003, 3.3747244800000002, 3.42008368,
        3.51080208, 3.6060564, 3.6831670400000003, 3.73759808, 3.81924464,
        3.88728344, 3.95532224, 4.0596484, 4.14583088, 4.254692960000001,
        4.30458808, 4.38169872, 4.5087044800000005, 4.52684816, 4.649318,
        4.70828496, 4.7853956, 4.83529072, 4.93054504, 5.0121916, 5.07569448,
        5.14373328, 5.17548472, 5.22084392, 5.245035493333333)
    CONTROLL = (
        1100, 1104, 1108, 1112, 1116, 1120, 1124, 1128, 1132, 1136, 1140, 1144,
        1148, 1152, 1156, 1160, 1164, 1168, 1172, 1176, 1180, 1184, 1188, 1192,
        1196, 1200, 1204, 1208, 1212, 1216, 1220, 1224, 1228, 1232, 1236, 1240,
        1244, 1248, 1252, 1256, 1260, 1264, 1268, 1272, 1276, 1280, 1284, 1288,
        1292, 1296, 1300, 1304, 1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336,
        1340, 1344, 1348, 1352, 1356, 1360, 1364, 1368, 1372, 1376, 1380, 1384,
        1388, 1392, 1396, 1400, 1404, 1408, 1412, 1416, 1420, 1424, 1428, 1432,
        1436, 1440, 1444, 1448, 1452, 1456, 1460, 1464, 1468, 1472, 1476, 1480,
        1484, 1488, 1492, 1496, 1500, 1504, 1508, 1512, 1516, 1520, 1524, 1528,
        1532, 1536, 1540, 1544, 1548, 1552, 1556, 1560, 1564, 1568, 1572, 1576,
        1580, 1584, 1588, 1592, 1596, 1600, 1604, 1608, 1612, 1616, 1620, 1624,
        1628, 1632, 1636, 1640, 1644, 1648, 1652, 1656, 1660, 1664, 1668, 1672,
        1676, 1680, 1684, 1688, 1692, 1696, 1700, 1704, 1708, 1712, 1716, 1720,
        1724, 1728, 1732, 1736, 1740, 1744, 1748, 1752, 1756, 1760, 1764, 1768,
        1772, 1776, 1780, 1784, 1788, 1792, 1796, 1800, 1804, 1808, 1812, 1816,
        1820, 1824, 1828, 1832, 1836, 1840, 1844, 1848, 1852, 1856, 1860, 1864,
        1868, 1872, 1876, 1880, 1884, 1888, 1892, 1896, 1900)

    def __init__(self):
        self.motor_init()
        listener = rospy.Subscriber(
            'Motors_Force', Float64MultiArray, self.callback)

    def motor_init(self, freq=224):
        # Set PWM frequency
        freq_raw = round(25000000 / (4096 * freq)) - 1
        print(f'freq ram: {freq_raw}')
        '''
        freq = 224 will make one step in pca9685 equal to 1e-6 (s).
        If want to change the frequency, need to set pca9685 to sleep mode
        '''
        self.set_sleep(self.pca9685_addr)
        self.bus.write_byte_data(self.pca9685_addr, 0xFE, freq_raw)
        self.unset_sleep(self.pca9685_addr)


        for i in range(16):
            self.set_PWM_ON(self.pca9685_addr, i, 0)
        for i in range(6):
            self.set_motor(i, 1500)    # send start signal
        #self.set_motor(1, 1500)

        ''' Check the desired frequnency is written to register '''
        # print('freq signal: ' + str(self.bus.read_byte_data(self.pca9685_addr, 0xFE)))

    @classmethod
    def callback(cls, data):
        #print(type(data.data))
        cmd = np.interp(data.data, cls.FORCE, cls.CONTROLL)
        for i in range(6):
            cls.set_motor(i, cmd[i])

        print(f'Total Force:    {data.data}')
        print(f'Final controll: {cmd}')
        # cls.set_motor(1, cmd[1])
        # rospy.loginfo('motor %d final control: %d', 1, cmd[1])
        # rospy.loginfo(data)

    ''' Set a fuse for motor max controlling '''
    @staticmethod
    def fuse(val):
        if val < 1104:      # 1104
            return 1104
        if val > 1896:      # 1896
            return 1896
        return int(val)

    ''' Mapping the datasheet control signal to real value '''
    ''' In this version, we assume there is only offset from the stop region '''
    @staticmethod
    def correction(val):
        if 1472 <= val <= 1528:
            return 1490             # [1472-1528] -> [1469-1509]
        elif val > 1528:
            return val-18           # 1529 -> 1511
        elif val < 1472:
            return val-4            # 1571 -> 1568

    ''' Send motor controll signal '''
    @classmethod
    def set_motor(cls, motor, val):
        val = cls.fuse(val)
        val = cls.correction(val)
        cls.set_PWM_OFF(cls.pca9685_addr, motor, val)

    ''' Will be called if press ctrl+C '''
    @classmethod
    def shutdown(cls):
        for _ in range(3):
            for i in range(6):
                cls.set_motor(i, 1500)
            time.sleep(0.02)
        #slow = [1400, 1420, 1440, 1460, 1480, 1500]
        #for j in range(6):
        #    for i in range(16):
        #        cls.set_motor(i, slow[j])
        #    time.sleep(0.02)
        print('\nStopped signal sent')

    '''
    function from Ref
    '''
    @classmethod
    def set_PWM_ON(cls, addr, ch, value):
        low_byte_val = value & 0x00FF
        high_byte_val = (value & 0x0F00) >> 8
        reg_low_byte = 0x06 + 4 * ch
        cls.bus.write_i2c_block_data(addr, reg_low_byte, [low_byte_val])
        cls.bus.write_i2c_block_data(addr, reg_low_byte+1, [high_byte_val])

    @classmethod
    def set_PWM_OFF(cls, addr, ch, value):
        low_byte_val = value & 0x00FF
        high_byte_val = (value & 0x0F00) >> 8
        reg_low_byte = 0x08 + 4 * ch
        cls.bus.write_i2c_block_data(addr, reg_low_byte, [low_byte_val])
        cls.bus.write_i2c_block_data(addr, reg_low_byte + 1, [high_byte_val])

    @classmethod
    def set_sleep(cls, addr):
        reg_mode1 = 0x00
        sleep_bit = 0x01 << 4
        old_mode1_val = cls.bus.read_byte_data(addr, reg_mode1)
        cls.bus.write_i2c_block_data(
            addr, reg_mode1, [old_mode1_val | sleep_bit])

    @classmethod
    def unset_sleep(cls, addr):
        reg_mode1 = 0x00
        sleep_bit = 0x01 << 4
        old_mode1_val = cls.bus.read_byte_data(addr, reg_mode1)
        cls.bus.write_i2c_block_data(
            addr, reg_mode1, [old_mode1_val & ~(sleep_bit)])


def main(args):
    MotorController()
    rospy.init_node('Motor_Controller', anonymous=True)
    rospy.on_shutdown(MotorController.shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('bye')


if __name__ == '__main__':
    main(sys.argv)
