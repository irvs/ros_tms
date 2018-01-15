# Localizationのやり方

scripts配下のlocalize,visualize内の座標を編集し,
roslaunch tms_ss_niniebot_pozyx positioning_map.launch

# ninebot navigationの動かし方

'''
ssh odroid@192.168.12.22
sudo bash
date --set "2018/01/12 08:40:00"
roslaunch tms_rc_bot ninebot_navigation.launch
'''
