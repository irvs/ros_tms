#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>

#include "tms_ss_kinect_v2/ConvertToJointAngles.h"
#include "tms_ss_kinect_v2/pose_setting.h"

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

class ConvertJointAngles
{
  public:
    ConvertJointAngles(ros::NodeHandle &handle);
    bool calculate_jointangles(
        tms_ss_kinect_v2::ConvertToJointAngles::Request &req,
        tms_ss_kinect_v2::ConvertToJointAngles::Response &res);
    void run();
  private:

    ros::NodeHandle& nh;
};

ConvertJointAngles::ConvertJointAngles(ros::NodeHandle &handle) :
  nh(handle)
{
  return;
}

bool ConvertJointAngles::calculate_jointangles(
    tms_ss_kinect_v2::ConvertToJointAngles::Request &req,
    tms_ss_kinect_v2::ConvertToJointAngles::Response &res)
{
  res.joint_angles.position.resize(6);
  res.joint_angles.joint_angle.resize(34);
  Eigen::Vector3f j[JOINT_NUM];
  for (int i=0; i<JOINT_NUM; i++)
  {
    //if (req.skeleton.confidence[i] == 0)
    //{
    //  return;
    //}
    j[i] = Eigen::Vector3f(
        req.skeleton.position[i].x,
        req.skeleton.position[i].y,
        req.skeleton.position[i].z);
    res.joint_angles.joint_angle[i] = 0.0;
  }

  Eigen::Vector3f v(
      req.skeleton.position[SpineBase].x,
      req.skeleton.position[SpineBase].y,
      req.skeleton.position[SpineBase].z);
  Eigen::Quaternionf q(
      req.skeleton.orientation[SpineBase].w,
      req.skeleton.orientation[SpineBase].x,
      req.skeleton.orientation[SpineBase].y,
      req.skeleton.orientation[SpineBase].z);

  Eigen::Vector3f vec;
  Eigen::Vector3f x,y,z;
  Eigen::Quaternionf rot[2];

  res.joint_angles.position[0] = req.skeleton.position[SpineBase].x;
  res.joint_angles.position[1] = req.skeleton.position[SpineBase].y;
  res.joint_angles.position[2] = req.skeleton.position[SpineBase].z;

  x = Eigen::Vector3f::UnitX();
  y = Eigen::Vector3f::UnitY();
  vec = (j[HipLeft]-j[HipRight]).cross(j[SpineMid]-j[SpineBase]);
  res.joint_angles.position[5] = atan2(y.dot(vec),x.dot(vec));
  rot[0] = Eigen::AngleAxisf(res.joint_angles.position[5],x.cross(y));
  x = rot[0] * Eigen::Vector3f::UnitZ();
  y = rot[0] * Eigen::Vector3f::UnitX();
  vec = j[SpineMid]-j[SpineBase];
  res.joint_angles.position[4] = atan2(y.dot(vec),x.dot(vec));
  rot[1] = Eigen::AngleAxisf(res.joint_angles.position[4],x.cross(y));
  x = rot[1] * rot[0] * Eigen::Vector3f::UnitZ();
  y = rot[1] * rot[0] * -Eigen::Vector3f::UnitY();
  vec = j[SpineMid]-j[SpineBase];
  res.joint_angles.position[3] = atan2(y.dot(vec),x.dot(vec));

  /*-- Model1: for choreonoid  --*/
  //if (1)//req.skeleton.confidence[SpineMid] == 2 &&
  //    //req.skeleton.confidence[SpineBase] == 2 &&
  //    //req.skeleton.confidence[SpineShoulder] == 2 &&
  //    //req.skeleton.confidence[ShoulderLeft] == 2 &&
  //    //req.skeleton.confidence[ShoulderRight] == 2)
  //{
  //  // 0:WAIST_JOINT1
  //  z = (j[SpineMid]-j[SpineBase]).normalized();
  //  x = ((j[HipLeft]-j[HipRight]).cross(j[SpineMid]-j[SpineBase])).normalized();
  //  y = (j[HipLeft]-j[HipRight]).normalized();
  //  vec = (j[SpineShoulder]-j[SpineMid]).cross(j[ShoulderLeft]-j[ShoulderRight]);
  //  res.joint_angles.joint_angle[0] = atan2(y.dot(vec),x.dot(vec));
  //  rot[0] = Eigen::AngleAxisf(res.joint_angles.joint_angle[0],x.cross(y));
  //  // 1:WAIST_JOINT2
  //  x = rot[0] * (j[SpineMid]-j[SpineBase]).normalized();
  //  y = rot[0] * ((j[SpineMid]-j[SpineBase]).cross(j[HipRight]-j[HipLeft])).normalized();
  //  vec = j[SpineShoulder]-j[SpineMid];
  //  res.joint_angles.joint_angle[1] = atan2(y.dot(vec),x.dot(vec));
  //}
  if (1)
    //req.skeleton.confidence[SpineShoulder] == 2 &&
      //req.skeleton.confidence[ShoulderRight] == 2 &&
      //req.skeleton.confidence[SpineMid] == 2 &&
      //req.skeleton.confidence[ElbowRight] == 2 &&
      //req.skeleton.confidence[WristRight] == 2)
  {
    // 2:R_ARM_JOINT1
    x = ((j[SpineMid]-j[SpineShoulder]).cross(j[ShoulderRight]-j[SpineShoulder])).normalized();
    y = (j[SpineShoulder]-j[SpineMid]).normalized();
    vec = (j[SpineShoulder]-j[ShoulderRight]).cross(j[ElbowRight]-j[ShoulderRight]);
    res.joint_angles.joint_angle[2] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(res.joint_angles.joint_angle[2],x.cross(y));
    // 3:R_ARM_JOINT2
    x = rot[0] * (j[SpineMid]-j[SpineShoulder]).normalized();
    y = rot[0] * (j[SpineShoulder]-j[ShoulderRight]).normalized();
    vec = j[ElbowRight]-j[ShoulderRight];
    res.joint_angles.joint_angle[3] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(res.joint_angles.joint_angle[3],x.cross(y));
    // 4:R_ARM_JOINT3
    x = rot[1] * rot[0] * (j[ShoulderRight]-j[SpineShoulder]).normalized();
    y = rot[1] * rot[0] * ((j[ShoulderRight]-j[SpineShoulder]).cross(j[SpineMid]-j[SpineShoulder])).normalized();
    vec = (j[WristRight]-j[ElbowRight]).cross(j[ShoulderRight]-j[ElbowRight]);
    res.joint_angles.joint_angle[4] = atan2(y.dot(vec),x.dot(vec));
    // 5:R_ARM_JOINT4
    z = (j[ShoulderRight]-j[ElbowRight]).cross(j[WristRight]-j[ElbowRight]);
    y = (z.cross(j[ElbowRight]-j[ShoulderRight])).normalized();
    x = (j[ElbowRight]-j[ShoulderRight]).normalized();
    vec = j[WristRight]-j[ElbowRight];
    res.joint_angles.joint_angle[5] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)//req.skeleton.confidence[SpineShoulder] == 2 &&
      //req.skeleton.confidence[ShoulderLeft] == 2 &&
      //req.skeleton.confidence[SpineMid] == 2 &&
      //req.skeleton.confidence[ElbowLeft] == 2 &&
      //req.skeleton.confidence[WristLeft] == 2)
  {
    // 10:L_ARM_JOINT1
    z = j[ShoulderLeft]-j[SpineShoulder];
    x = (j[SpineMid]-j[SpineShoulder]).normalized();
    y = (z.cross(j[SpineMid]-j[SpineShoulder])).normalized();
    vec = j[ElbowLeft]-j[ShoulderLeft];
    res.joint_angles.joint_angle[10] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(res.joint_angles.joint_angle[10],x.cross(y));
    // 11:L_ARM_JOINT2
    x = rot[0] * (j[SpineMid]-j[SpineShoulder]).normalized();
    y = rot[0] * (j[ShoulderLeft]-j[SpineShoulder]).normalized();
    vec = j[ElbowLeft]-j[ShoulderLeft];
    res.joint_angles.joint_angle[11] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(res.joint_angles.joint_angle[11],x.cross(y));
    // 12:L_ARM_JOINT3
    x = rot[1] * rot[0] * (j[ShoulderLeft]-j[SpineShoulder]).normalized();
    y = rot[1] * rot[0] * ((j[ShoulderLeft]-j[SpineShoulder]).cross(j[SpineMid]-j[SpineShoulder])).normalized();
    vec = (j[ShoulderLeft]-j[ElbowLeft]).cross(j[WristLeft]-j[ElbowLeft]);
    res.joint_angles.joint_angle[12] = atan2(y.dot(vec),x.dot(vec));
    // 13:L_ARM_JOINT4
    z = (j[ShoulderLeft]-j[ElbowLeft]).cross(j[WristLeft]-j[ElbowLeft]);
    x = (j[ElbowLeft]-j[ShoulderLeft]).normalized();
    y = (z.cross(j[ElbowLeft]-j[ShoulderLeft])).normalized();
    vec = j[WristLeft]-j[ElbowLeft];
    res.joint_angles.joint_angle[13] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)
  {
    // 20:R_LEG_JOINT1
    z = j[SpineBase]-j[HipRight];
    x = (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeRight]-j[HipRight];
    res.joint_angles.joint_angle[20] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(res.joint_angles.joint_angle[20],x.cross(y));
    // 21:R_LEG_JOINT2
    z = rot[0] * (j[HipRight]-j[HipLeft]).cross(j[SpineBase]-j[SpineMid]);
    x = rot[0] * (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeRight]-j[HipRight];
    res.joint_angles.joint_angle[21] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(res.joint_angles.joint_angle[21],x.cross(y));
    // 22:R_LEG_JOINT3
    x = rot[1] * rot[0] * (j[HipRight]-j[HipLeft]).normalized();
    y = rot[1] * rot[0] * ((j[HipRight]-j[HipLeft]).cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = (j[HipRight]-j[KneeRight]).cross(j[AnkleRight]-j[KneeRight]);
    res.joint_angles.joint_angle[22] = atan2(y.dot(vec),x.dot(vec));
    // 23:R_LEG_JOINT4
    z = (j[AnkleRight]-j[KneeRight]).cross(j[HipRight]-j[KneeRight]);
    x = (j[KneeRight]-j[HipRight]).normalized();
    y = (z.cross(j[KneeRight]-j[HipRight])).normalized();
    vec = j[AnkleRight]-j[KneeRight];
    res.joint_angles.joint_angle[23] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)
  {
    // 27:L_LEG_JOINT1
    z = j[HipLeft]-j[SpineBase];
    x = (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeLeft]-j[HipLeft];
    res.joint_angles.joint_angle[27] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(res.joint_angles.joint_angle[27],x.cross(y));
    // 28:L_LEG_JOINT2
    z = rot[0] * (j[HipRight]-j[HipLeft]).cross(j[SpineBase]-j[SpineMid]);
    x = rot[0] * (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeLeft]-j[HipLeft];
    res.joint_angles.joint_angle[28] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(res.joint_angles.joint_angle[28],x.cross(y));
    // 29:L_LEG_JOINT3
    z = j[HipLeft]-j[KneeLeft];
    x = rot[1] * rot[0] * (j[HipRight]-j[HipLeft]).normalized();
    y = (z.cross(x)).normalized();
    vec = (j[HipLeft]-j[KneeLeft]).cross(j[AnkleLeft]-j[KneeLeft]);
    res.joint_angles.joint_angle[29] = atan2(y.dot(vec),x.dot(vec));
    // 30:L_LEG_JOINT4
    z = (j[AnkleLeft]-j[KneeLeft]).cross(j[HipLeft]-j[KneeLeft]);
    x = (j[KneeLeft]-j[HipLeft]).normalized();
    y = (z.cross(j[KneeLeft]-j[HipLeft])).normalized();
    vec = j[AnkleLeft]-j[KneeLeft];
    res.joint_angles.joint_angle[30] = atan2(y.dot(vec),x.dot(vec));
  }
  ROS_INFO("ConvertToJointAngles: Service succeed to %d", req.skeleton.user_id);
  return true;
}

void ConvertJointAngles::run()
{
  ros::ServiceServer service = nh.advertiseService("convert_to_jointangles",
      &ConvertJointAngles::calculate_jointangles,this);
  ros::spin();
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"calc_joint_angles");
  ros::NodeHandle nh;

  ConvertJointAngles converter(nh);

  converter.run();
  return 0;
}
