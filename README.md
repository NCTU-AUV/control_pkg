# AUV Control ROS Package
This is a ros package for AUV control group.

## Usage
### Clone the package
``` bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/NCTU-AUV/control_pkg.git
$ cd ../ && catkin_make
```


### Rosrun the package
**NOTE** `$source ~/catkin_ws/devel/setup.bash` for *every new* terminal.

Total needs 6 (+3) terminals
- `$ rosrun control_pkg imu9250.py`
- `$ rosrun control_pkg bar30.py`
- `$ rosrun control_pkg pid_attitude.py`
- `$ rosrun control_pkg pid_depth.py`
- `$ rosrun control_pkg motor_force_summation.py`
- `$ rosrun control_pkg motor_output.py`

*Optional* Manual control using command.
- `$ rosrun control_pkg cmd_controller.py`
- `$ rosrun control_pkg joy_cmd_emitter.py`
- `$ rosrun control_pkg keyboard_cmd_emitter.py`

## Reminder
### Sensor Nodes
1. The bridge node, which receives arduino data from serial port, **cannot identify** two arduinos. ie. needs to specify the ports. Could find a way to identify them.
2. The depth node (bar30.py) may need a linear conversion to unstandable value.  
    e.g.
    ``` python
    def cal(self, x):
        a = 5.667
        b = -1127.97
        
        # y = ax+b
        return a*x+b
    ```
3.Though we did combine collecting attitude and depth data on the same arduino, the freshing rate is too slow as a result.

### Non ROS
- reset_arduino.py  
    By connecting **pin 37** to arduio interrupt pin, we can reset it on remote.