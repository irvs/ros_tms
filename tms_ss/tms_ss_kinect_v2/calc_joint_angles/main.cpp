#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>

#include "tms_ss_kinect_v2/SkeletonArray.h"
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
    void callback(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg);
    void run();
  private:
    void setJointAngles(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg);

    ros::NodeHandle& nh;
    ros::Publisher pub;
    tms_ss_kinect_v2::pose_setting cnoid_skeleton;
};

ConvertJointAngles::ConvertJointAngles(ros::NodeHandle &handle) :
  nh(handle)
{
  cnoid_skeleton.position.resize(6);
  cnoid_skeleton.joint_angle.resize(34);
  pub = nh.advertise<tms_ss_kinect_v2::pose_setting>("skeleton_pose_set", 1);
  return;
}

void ConvertJointAngles::callback(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg)
{
  ROS_INFO("Got skeleton: [id = %d]", msg->data[0].user_id);

  setJointAngles(msg);

  return;
}

void ConvertJointAngles::setJointAngles(const tms_ss_kinect_v2::SkeletonArray::ConstPtr& msg)
{
  Eigen::Vector3f j[JOINT_NUM];
  for (int i=0; i<JOINT_NUM; i++)
  {
    //if (msg->data[0].confidence[i] == 0)
    //{
    //  return;
    //}
    j[i] = Eigen::Vector3f(
        msg->data[0].position[i].x,
        msg->data[0].position[i].y,
        msg->data[0].position[i].z);
    cnoid_skeleton.joint_angle[i] = 0.0;
  }

  Eigen::Vector3f v(
      msg->data[0].position[SpineBase].x,
      msg->data[0].position[SpineBase].y,
      msg->data[0].position[SpineBase].z);
  Eigen::Quaternionf q(
      msg->data[0].orientation[SpineBase].w,
      msg->data[0].orientation[SpineBase].x,
      msg->data[0].orientation[SpineBase].y,
      msg->data[0].orientation[SpineBase].z);

  Eigen::Vector3f vec;
  Eigen::Vector3f x,y,z;
  Eigen::Quaternionf rot[2];

  cnoid_skeleton.position[0] = msg->data[0].position[SpineBase].x;
  cnoid_skeleton.position[1] = msg->data[0].position[SpineBase].y;
  cnoid_skeleton.position[2] = msg->data[0].position[SpineBase].z;

  x = Eigen::Vector3f::UnitX();
  y = Eigen::Vector3f::UnitY();
  vec = (j[HipLeft]-j[HipRight]).cross(j[SpineMid]-j[SpineBase]);
  cnoid_skeleton.position[5] = atan2(y.dot(vec),x.dot(vec));
  rot[0] = Eigen::AngleAxisf(cnoid_skeleton.position[5],x.cross(y));
  x = rot[0] * Eigen::Vector3f::UnitZ();
  y = rot[0] * Eigen::Vector3f::UnitX();
  vec = j[SpineMid]-j[SpineBase];
  cnoid_skeleton.position[4] = atan2(y.dot(vec),x.dot(vec));
  rot[1] = Eigen::AngleAxisf(cnoid_skeleton.position[4],x.cross(y));
  x = rot[1] * rot[0] * Eigen::Vector3f::UnitZ();
  y = rot[1] * rot[0] * -Eigen::Vector3f::UnitY();
  vec = j[SpineMid]-j[SpineBase];
  cnoid_skeleton.position[3] = atan2(y.dot(vec),x.dot(vec));

  if (1)//msg->data[0].confidence[SpineMid] == 2 &&
      //msg->data[0].confidence[SpineBase] == 2 &&
      //msg->data[0].confidence[SpineShoulder] == 2 &&
      //msg->data[0].confidence[ShoulderLeft] == 2 &&
      //msg->data[0].confidence[ShoulderRight] == 2)
  {
    // 0:WAIST_JOINT1
    z = (j[SpineMid]-j[SpineBase]).normalized();
    x = ((j[HipLeft]-j[HipRight]).cross(j[SpineMid]-j[SpineBase])).normalized();
    y = (j[HipLeft]-j[HipRight]).normalized();
    vec = (j[SpineShoulder]-j[SpineMid]).cross(j[ShoulderLeft]-j[ShoulderRight]);
    cnoid_skeleton.joint_angle[0] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[0],x.cross(y));
    // 1:WAIST_JOINT2
    x = rot[0] * (j[SpineMid]-j[SpineBase]).normalized();
    y = rot[0] * ((j[SpineMid]-j[SpineBase]).cross(j[HipRight]-j[HipLeft])).normalized();
    vec = j[SpineShoulder]-j[SpineMid];
    cnoid_skeleton.joint_angle[1] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)
    //msg->data[0].confidence[SpineShoulder] == 2 &&
      //msg->data[0].confidence[ShoulderRight] == 2 &&
      //msg->data[0].confidence[SpineMid] == 2 &&
      //msg->data[0].confidence[ElbowRight] == 2 &&
      //msg->data[0].confidence[WristRight] == 2)
  {
    // 2:R_ARM_JOINT1
    x = ((j[SpineMid]-j[SpineShoulder]).cross(j[ShoulderRight]-j[SpineShoulder])).normalized();
    y = (j[SpineShoulder]-j[SpineMid]).normalized();
    vec = (j[SpineShoulder]-j[ShoulderRight]).cross(j[ElbowRight]-j[ShoulderRight]);
    cnoid_skeleton.joint_angle[2] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[2],x.cross(y));
    // 3:R_ARM_JOINT2
    x = rot[0] * (j[SpineMid]-j[SpineShoulder]).normalized();
    y = rot[0] * (j[SpineShoulder]-j[ShoulderRight]).normalized();
    vec = j[ElbowRight]-j[ShoulderRight];
    cnoid_skeleton.joint_angle[3] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[3],x.cross(y));
    // 4:R_ARM_JOINT3
    x = rot[1] * rot[0] * (j[ShoulderRight]-j[SpineShoulder]).normalized();
    y = rot[1] * rot[0] * ((j[ShoulderRight]-j[SpineShoulder]).cross(j[SpineMid]-j[SpineShoulder])).normalized();
    vec = (j[WristRight]-j[ElbowRight]).cross(j[ShoulderRight]-j[ElbowRight]);
    cnoid_skeleton.joint_angle[4] = atan2(y.dot(vec),x.dot(vec));
    // 5:R_ARM_JOINT4
    z = (j[ShoulderRight]-j[ElbowRight]).cross(j[WristRight]-j[ElbowRight]);
    y = (z.cross(j[ElbowRight]-j[ShoulderRight])).normalized();
    x = (j[ElbowRight]-j[ShoulderRight]).normalized();
    vec = j[WristRight]-j[ElbowRight];
    cnoid_skeleton.joint_angle[5] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)//msg->data[0].confidence[SpineShoulder] == 2 &&
      //msg->data[0].confidence[ShoulderLeft] == 2 &&
      //msg->data[0].confidence[SpineMid] == 2 &&
      //msg->data[0].confidence[ElbowLeft] == 2 &&
      //msg->data[0].confidence[WristLeft] == 2)
  {
    // 10:L_ARM_JOINT1
    z = j[ShoulderLeft]-j[SpineShoulder];
    x = (j[SpineMid]-j[SpineShoulder]).normalized();
    y = (z.cross(j[SpineMid]-j[SpineShoulder])).normalized();
    vec = j[ElbowLeft]-j[ShoulderLeft];
    cnoid_skeleton.joint_angle[10] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[10],x.cross(y));
    // 11:L_ARM_JOINT2
    x = rot[0] * (j[SpineMid]-j[SpineShoulder]).normalized();
    y = rot[0] * (j[ShoulderLeft]-j[SpineShoulder]).normalized();
    vec = j[ElbowLeft]-j[ShoulderLeft];
    cnoid_skeleton.joint_angle[11] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[11],x.cross(y));
    // 12:L_ARM_JOINT3
    x = rot[1] * rot[0] * (j[ShoulderLeft]-j[SpineShoulder]).normalized();
    y = rot[1] * rot[0] * ((j[ShoulderLeft]-j[SpineShoulder]).cross(j[SpineMid]-j[SpineShoulder])).normalized();
    vec = (j[ShoulderLeft]-j[ElbowLeft]).cross(j[WristLeft]-j[ElbowLeft]);
    cnoid_skeleton.joint_angle[12] = atan2(y.dot(vec),x.dot(vec));
    // 13:L_ARM_JOINT4
    z = (j[ShoulderLeft]-j[ElbowLeft]).cross(j[WristLeft]-j[ElbowLeft]);
    x = (j[ElbowLeft]-j[ShoulderLeft]).normalized();
    y = (z.cross(j[ElbowLeft]-j[ShoulderLeft])).normalized();
    vec = j[WristLeft]-j[ElbowLeft];
    cnoid_skeleton.joint_angle[13] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)
  {
    // 20:R_LEG_JOINT1
    z = j[SpineBase]-j[HipRight];
    x = (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeRight]-j[HipRight];
    cnoid_skeleton.joint_angle[20] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[20],x.cross(y));
    // 21:R_LEG_JOINT2
    z = rot[0] * (j[HipRight]-j[HipLeft]).cross(j[SpineBase]-j[SpineMid]);
    x = rot[0] * (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeRight]-j[HipRight];
    cnoid_skeleton.joint_angle[21] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[21],x.cross(y));
    // 22:R_LEG_JOINT3
    x = rot[1] * rot[0] * (j[HipRight]-j[HipLeft]).normalized();
    y = rot[1] * rot[0] * ((j[HipRight]-j[HipLeft]).cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = (j[HipRight]-j[KneeRight]).cross(j[AnkleRight]-j[KneeRight]);
    cnoid_skeleton.joint_angle[22] = atan2(y.dot(vec),x.dot(vec));
    // 23:R_LEG_JOINT4
    z = (j[AnkleRight]-j[KneeRight]).cross(j[HipRight]-j[KneeRight]);
    x = (j[KneeRight]-j[HipRight]).normalized();
    y = (z.cross(j[KneeRight]-j[HipRight])).normalized();
    vec = j[AnkleRight]-j[KneeRight];
    cnoid_skeleton.joint_angle[23] = atan2(y.dot(vec),x.dot(vec));
  }
  if (1)
  {
    // 27:L_LEG_JOINT1
    z = j[HipLeft]-j[SpineBase];
    x = (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeLeft]-j[HipLeft];
    cnoid_skeleton.joint_angle[27] = atan2(y.dot(vec),x.dot(vec));
    rot[0] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[27],x.cross(y));
    // 28:L_LEG_JOINT2
    z = rot[0] * (j[HipRight]-j[HipLeft]).cross(j[SpineBase]-j[SpineMid]);
    x = rot[0] * (j[SpineBase]-j[SpineMid]).normalized();
    y = (z.cross(j[SpineBase]-j[SpineMid])).normalized();
    vec = j[KneeLeft]-j[HipLeft];
    cnoid_skeleton.joint_angle[28] = atan2(y.dot(vec),x.dot(vec));
    rot[1] = Eigen::AngleAxisf(cnoid_skeleton.joint_angle[28],x.cross(y));
    // 29:L_LEG_JOINT3
    z = j[HipLeft]-j[KneeLeft];
    x = rot[1] * rot[0] * (j[HipRight]-j[HipLeft]).normalized();
    y = (z.cross(x)).normalized();
    vec = (j[HipLeft]-j[KneeLeft]).cross(j[AnkleLeft]-j[KneeLeft]);
    cnoid_skeleton.joint_angle[29] = atan2(y.dot(vec),x.dot(vec));
    // 30:L_LEG_JOINT4
    z = (j[AnkleLeft]-j[KneeLeft]).cross(j[HipLeft]-j[KneeLeft]);
    x = (j[KneeLeft]-j[HipLeft]).normalized();
    y = (z.cross(j[KneeLeft]-j[HipLeft])).normalized();
    vec = j[AnkleLeft]-j[KneeLeft];
    cnoid_skeleton.joint_angle[30] = atan2(y.dot(vec),x.dot(vec));
  }
  return;
}

void ConvertJointAngles::run()
{
  pub.publish(cnoid_skeleton);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"calc_joint_angles");
  ros::NodeHandle nh;

  ConvertJointAngles converter(nh);

  ros::Subscriber sub = nh.subscribe("integrated_skeleton_stream",1,
      &ConvertJointAngles::callback,&converter);
  while (ros::ok())
  {
    converter.run();
    ros::spinOnce();
  }
  return 0;
}
