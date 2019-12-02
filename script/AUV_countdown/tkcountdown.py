#!/usr/bin/env python
import Tkinter as tk
import rospy
import time
from std_msgs.msg import Int32
class ExampleApp(tk.Tk):
    def __init__(self,number):
        tk.Tk.__init__(self)
        rospy.init_node('countdownGUI', anonymous=True)
        self.countdowner = rospy.Publisher('/AUVmanage/countdowner',Int32,queue_size=1)
        time.sleep(0.5)
        self.num = Int32(data = number)
        self.countdowner.publish(self.num)
        self.label = tk.Label(self, text="", width=10)
        self.label.pack()
        self.remaining = 0
        self.countdown(number)

    def countdown(self, remaining = None):
      #self.countdowner.publish(self.num)
      if remaining is not None:
        self.remaining = remaining

      if self.remaining <= 0:
        self.label.configure(text="time's up!")
      else:
        self.label.configure(text=str(self.remaining))
        self.remaining = self.remaining - 0.1
        self.after(100, self.countdown)

if __name__ == "__main__":

    number = input("num:")
    print(number)
    app = ExampleApp(number)
    app.mainloop()