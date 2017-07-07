# 4台のLRFによるトラッキング

## 0. Precondition
IPアドレスは以下の通りで,ROSMASTER_URIなどの設定はそれぞれの端末で完了しているとする.

| Name | IP Address |
|:-----:|:-----:|
| Server | 192.168.11.2 |
| Odroid_1 | 192.168.11.11 |
| Odroid_2 | 192.168.11.12 |
| Odroid_3 | 192.168.11.13 |
| Odroid_4 | 192.168.11.14 |

## 1. Prepare Server PC
```bash
roscore
ssh odroid@192.168.11.11  # Password:odroid
ssh odroid@192.168.11.12  # Password:odroid
ssh odroid@192.168.11.13  # Password:odroid
ssh odroid@192.168.11.14  # Password:odroid
```

## 2. Prepare Odroid
それぞれのsshの画面で,
```bash
roslaunch tms_ss_pot urg*.launch  # アスタリスクは1~4の番号
```

## 3. Run Tracker
Server PC上で,
```bash
roslaunch tms_ss_pot tracker_map.launch
```

## Ex. Laser Calibration
LRFの設置位置を調整するときは,
```bash
roslaunch tms_ss_pot laser_calibration.launch
```
で,LaserのMarkerArrayを表示して調整.
