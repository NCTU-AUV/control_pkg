'''
Intro:
data: leftTime (var, in second)
data: listbox_signals.curselection() (func, return 0-2, 0 for 'Signal 1')
1. Not connected to ROS Topic
2. Not in class hierachy

Ref:
[ListBox]https://morvanzhou.github.io/tutorials/python-basic/tkinter/2-03-listbox/
[Entry]https://www.runoob.com/python/python-tkinter-entry.html
'''

# 引入套件
import tkinter as tk
import time

def countdown(remaining = None):
  #self.countdowner.publish(self.num)
  if remaining is not None:
    remaining = remaining
  if remaining <= 0:
    label_leftTime.configure(text="time's up!")
    button_start.config(state='normal')
  else:
    label_leftTime.configure(text=str(remaining))
    remaining = remaining - 0.1
    window.after(100, countdown, remaining)

def print_result():
    try:
        leftTime_input = float(entry_countdownTime.get())
        tmpStr = str(leftTime_input)+'s '+str(listbox_signals.curselection()) 
        signal_selection.set(tmpStr)
        button_start.config(state='disabled')
        countdown(leftTime_input)
    except ValueError:
        label_leftTime.config(text='Nonnumeric input!!')   

# 建立主視窗和 Frame（把元件變成群組的容器）
window = tk.Tk()

remaining = 0
signal_list = tk.StringVar()
signal_list.set(('Signal 1', 'Signal 2', 'Signal 3'))
signal_selection = tk.StringVar()
leftTime_input = 0

# 設定視窗標題、大小和背景顏色
window.title('PO Coundown App')
window.geometry('500x140')
window.configure(background='white')

# Create Frame
frame_control = tk.Frame(window)
frame_leftTime = tk.Frame(frame_control)
frame_selSignal = tk.Frame(frame_control)
frame_result = tk.Frame(frame_control)
# Create Labels
label_leftTime = tk.Label(window, width=20, height=3, bg='white', text='Time left...')
label_selection = tk.Label(frame_result, width=20, textvariable=signal_selection)  # for Debug
label_info_inputTime = tk.Label(frame_leftTime, width=10, text='Left time')
label_info_selSignal = tk.Label(frame_selSignal, width=10, text='Signal mode')
label_info_result = tk.Label(frame_result, width=10, text='Check result')
# Create Entry to input countdown time
entry_countdownTime = tk.Entry(frame_leftTime, width=20, bd=5)
# Create Button
button_start = tk.Button(window, width=20, text='Click', command=print_result)
# Create ListBox
listbox_signals = tk.Listbox(frame_selSignal, width=20, height=5, listvariable=signal_list)

# Frame pack for typesetting
label_leftTime.pack(side='left')

frame_control.pack(side='left')
frame_leftTime.pack(side='top')
label_info_inputTime.pack(side='left')
entry_countdownTime.pack(side='left')
frame_selSignal.pack(side='top')
label_info_selSignal.pack(side='left')
listbox_signals.pack(side='left')
frame_result.pack(side='top')
label_info_result.pack(side='left')
label_selection.pack(side='left')

button_start.pack(side='left')

# Console port
# number = float(input('enter num:'))

# 運行主程式
window.mainloop()