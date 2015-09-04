#ifndef _CALC_JOINT_ANGLES_H_
#define _CALC_JOINT_ANGLES_H_

#include <vector>
#include <map>
#include <Eigen/Eigen>

#include <tms_ss_kinect_v2/Skeleton.h>

typedef enum {
  SpineBase,
  SpineMid,
  Neck,
  Head,
  ShoulderLeft,
  ElbowLeft,
  WristLeft,
  HandLeft,
  ShoulderRight,
  ElbowRight,
  WristRight,
  HandRight,
  HipLeft,
  KneeLeft,
  AnkleLeft,
  FootLeft,
  HipRight,
  KneeRight,
  AnkleRight,
  FootRight,
  SpineShoulder,
  HandTipLeft,
  ThumbLeft,
  HandTipRight,
  ThumbRight,
  JOINT_NUM
} JOINT_NAME;

const char* kJointName[] = {
  //"WAIST_JOINT1",
  //"WAIST_JOINT2",
  "R_ARM_JOINT1",
  "R_ARM_JOINT2",
  "R_ARM_JOINT3",
  "R_ARM_JOINT4",
  "R_ARM_JOINT5",
  "R_ARM_JOINT6",
  "R_ARM_JOINT7",
  //"R_ARM_JOINT8",
  "L_ARM_JOINT1",
  "L_ARM_JOINT2",
  "L_ARM_JOINT3",
  "L_ARM_JOINT4",
  "L_ARM_JOINT5",
  "L_ARM_JOINT6",
  "L_ARM_JOINT7",
  //"L_ARM_JOINT8",
  "NECK_JOINT0",
  "NECK_JOINT1",
  "NECK_JOINT2",
  "R_LEG_JOINT1",
  "R_LEG_JOINT2",
  "R_LEG_JOINT3",
  "R_LEG_JOINT4",
  "R_LEG_JOINT5",
  "R_LEG_JOINT6",
  //"R_LEG_JOINT7",
  "L_LEG_JOINT1",
  "L_LEG_JOINT2",
  "L_LEG_JOINT3",
  "L_LEG_JOINT4",
  "L_LEG_JOINT5",
  "L_LEG_JOINT6"
  //"L_LEG_JOINT7",
};

const int kJointDoF = 29;

template <class T>
void calcJointAnglesForModel01(
    const tms_ss_kinect_v2::Skeleton &in,
    std::map<std::string, T> &out)
{
  std::vector<Eigen::Matrix<T,3,1> > j;
  for (int i=0; i<JOINT_NUM; i++)
  {
    j[i] = Eigen::Matrix<T, 3, 1>(
        in.position[i].x,
        in.position[i].y,
        in.position[i].z);
  }
  for (int i=0; i<kJointDoF; i++)
  {
    out[kJointName[i]] = 1.56;
  }
  return;
}

#endif //_CALC_JOINT_ANGLES_H_
