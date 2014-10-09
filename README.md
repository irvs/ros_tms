ros-tms
=======
# ROS-TMS
* version : v3.3.1
* git tag v3.1.4 808633ca2 / v3.0.0 e3b481e9c
* date : 2014.6.30 (since 2012.5.1 ROS-TMS / since 2005.11.1 TMS)
* director :

 Prof. Ryo Kurazume, Assistant Prof. Tokuo Tsuji

* developer :

 2014 : Yoonseok Pyo, Yuka Hashiguchi, Tetsuro Oishi, Kazuya Suginohara, Kazuto Nakashima, Yuta Watanabe, Daisuke Inada, Toshio Shigekane, Tokuo Tsuji, Omar Alaoui

 2013 : Yoonseok Pyo, Akihiro Nagata, Shunya Kuwahata, Kouhei Nakashima, Kazuya Kusaka, Yuka Hashiguchi, Tetsuro Oishi, Kazuya Suginohara, Tokuo Tsuji

 2012 : Yoonseok Pyo, Masahide Tanaka, Akihiro Nagata, Tokuo Tsuji

* dependency software :
 * Ubuntu 14.04 LTS 64BIT
 * Ros Indigo
 * Choreonoid 1.4 
 * graspPlugin for Choreonoid
 * MySQL 5.6.16
 * libqwt-dev, libqwt6, 
 * opencv 2.4.8 
 * libpcl-1.7-all libopenni2-0 etc...

 * pcl_ros
 * ARIA 2.8.1 (http://robots.mobilerobots.com/wiki/ARIA)

* repository :  http://orion.ait.kyushu-u.ac.jp/HRSG/ros_tms
* wiki : http://orion.ait.kyushu-u.ac.jp/HRSG/ros_tms/wikis/home

# How to install the ROS-TMS packages
* [how to install the ROS-TMS packages](install)

# How to run  the ROS-TMS packages
* [run_ros_tms](run_ros_tms)

# ROS-TMS meta-package information
* meta-package : 1
* package : 52
* node : 93
* lib for cnoid : 6
* messages : 43
* services : 70 

```
rospack list-names | grep tms
```

# TMS: Town Management System
**ROS-TMS** is new information management system for the informationally structured environment. This system enables to integrate various data from distributed sensors, store them to an online database, and plan service motion of a robot using real-time information about the surroundings.

# ROS-TMS architecture

* TMS-UR  : User Request

* TMS-TS  : Task Scheduler

* TMS-SA  : State Analyzer 
* TMS-SS  : Sensor System 
* TMS-SD  : Sensor Driver 

* TMS-RP  : Robot Planing 
* TMS-RS  : Robot Service
* TMS-RC  : Robot Control 

* TMS-DB  : Data Base

* TMS-MSG : TMS MESsage and service files
* TMS-DEV : TMS DEVelop tool


![architecture](http://orion.ait.kyushu-u.ac.jp/uploads/HRSG/ros_tms/98b813f85f/architecture.png)

![node](http://orion.ait.kyushu-u.ac.jp/uploads/HRSG/ros_tms/b9b0be1a72/node.png)

## tms_ur (0,0) : User Request 

## tms_ts (1,1) : Task Scheduler
* tms_ts_ts (1)

## tms_sa (4,5) : State Analyzer
* tms_sa_bas_graph (1)
* tms_sa_bas_person (1)
* tms_sa_bas_pub (1)
* tms_sa_missing_objects (2)

## tms_ss (19,47) : Sensor System
* tms_ss_fss_exe (8)
* tms_ss_fss_graph (1)
* tms_ss_his (2)
* tms_ss_ics (1)
* tms_ss_ods_capture (9)
* tms_ss_ods_change_detection (5)
* tms_ss_ods_color_segmentation (1)
* tms_ss_ods_correspondence_estimation (1)
* tms_ss_ods_cylinder (1)
* tms_ss_ods_face_detection (1)
* tms_ss_ods_hshistogram (1)
* tms_ss_ods_make_model (6)
* tms_ss_ods_normal_estimation (1)
* tms_ss_ods_person_detection (2)
* tms_ss_ods_register_capture (2)
* tms_ss_ods_robot_position (1)
* tms_ss_ods_skin_color (1)
* tms_ss_ods_trigger (1)
* tms_ss_ods_wagon (3)

## tms_sd (1,1) : Sensor Driver
* vicon_bridge (1)

## tms_rp (5,16) : Robot Planing
* tms_rp_action (2) lib for choreonoid
* tms_rp_rostms_plugin (4) lib for choreonoid
* tms_rp_rps_checker (2)
* tms_rp_rps_commander (4)
* tms_rp_rps_planner (4)

## tms_rc (11,18) : Robot Control
* tms_rc_blocked_function (2)
* tms_rc_katana (2)
* tms_rc_kobuki_control (1)
* tms_rc_pioneer (1)
* tms_rc_smartpal_action (1)
* tms_rc_smartpal_check (2)
* tms_rc_smartpal_control (2)
* tms_rc_smartpal_control_test (4)
* tms_rc_smartpal_tagreader (1)
* tms_rc_smartpal_tts (1)
* tms_rc_smartpal_virtual_control (1)

## tms_db (1,2) : Database
* tms_db_access (2)

## tms_dev (2,3) : Development tools
* tms_dev_rs (2)
* tms_dev_viewer (1)

## tms_msg (8,0) : Message and service files
* tms_msg_ur  0 messages,  0 services
* tms_msg_ts  1 messages,  2 services
* tms_msg_sa  1 messages,  4 services
* tms_msg_ss 23 messages, 10 services
* tms_msg_sd  0 messages,  0 services
* tms_msg_rp 14 messages, 25 services
* tms_msg_rc  3 messages, 11 services
* tms_msg_db  1 messages, 18 services
* total 43 messages, 70 services


