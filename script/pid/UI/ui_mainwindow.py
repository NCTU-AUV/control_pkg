#!/usr/bin/python3
import sys
from PyQt5 import QtWidgets, uic

from ui_pid_control import Ui_Dialog

# from auv_control.srv import pid_control 
# import rospy


class MainWindow(QtWidgets.QMainWindow, Ui_Dialog):
    def __init__(self, obj=None, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)

# if __name__ == '__main__':
app = QtWidgets.QApplication(sys.argv)

window = MainWindow()
window.show()
app.exec()
