# tms_rc_mimamorukun_control
---
## Minimal control program.
```
roslaunch tms_rc_mimamorukun_control minimal.launch
```
This node serve
* Subcrive topic for control **velocity**
* Publish topics for **odometry** and **tf** information

You must launch this file first at all.

If you want to change IP address and port setting, you must modify minimal.launch.
You can check IP address and port setting by connecting usb to Mbed(micro controller) in robot.


---
### Joystick control
```
roslaunch tms_rc_mimamorukun_control joyop.launch
```
Make sure to ``yocs-joyop`` is installed.
If not, run command below
```
sudo apt-get install ros-indigo-yocs-joyop
```

### Keyboard control
```
roslaunch tms_rc_mimamorukun_control keyop.launch
```
Make sure to ``kobuki_keyop`` is installed.
If not, run command below
```
sudo apt-get install ros-indigo-kobuki_keyop
```
