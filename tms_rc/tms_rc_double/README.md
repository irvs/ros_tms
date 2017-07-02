# 実行手順
以下のコマンドを実行
roscore  
rosrun tms_ss_vicon vicon_stream  
rosrun tms_rc_double double_control  
roslaunch tms_rc_double move_base_bsen.launch  

rviz /move_base_simple/goal:=/double_control/move_base_simple/goal  




### 注意
- /cmd_vel_mux/input/keyop　を発行することでDoubleが動く  
- 事前にDouble上のアプリケーションのROSMASTER_URIを，roscoreが起動しているPCのIPアドレスにしておく