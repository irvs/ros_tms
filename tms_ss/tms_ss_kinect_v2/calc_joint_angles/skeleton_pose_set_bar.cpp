//~ #include <tms_rp_controller.h>
#include <sstream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "skeleton_pose_set_bar.h"
#include <tms_ss_kinect_v2/pose_setting.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <cnoid/JointPath>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/MessageView>

#include <Grasp/VectorMath.h>
#include <Grasp/GraspController.h>

int argc;
char **argv;

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

const char *object_names[] = {
  "person_1",
  "person_2",
  "person_3",
  "person_4",
  "person_5",
  "person_6"
};


//------------------------------------------------------------------------------
SkeletonPoseBar* SkeletonPoseBar::instance()
{
  static SkeletonPoseBar* instance = new SkeletonPoseBar();
  return instance;
}

//------------------------------------------------------------------------------
SkeletonPoseBar::SkeletonPoseBar(): ToolBar("SkeletonPoseBar"),
                              mes(*MessageView::mainInstance()),
                              os(MessageView::mainInstance()->cout()),
                              tc(grasp::PlanBase::instance())
{
  addSeparator();
  addLabel(("=SkeletonPose="));
  addButton(("Set Pose"), ("Set Skeleton Pose"))->
    sigClicked().connect(bind(&SkeletonPoseBar::onSetPoseButtonClicked, this));

  return;
}

//------------------------------------------------------------------------------
SkeletonPoseBar::~SkeletonPoseBar()
{
  return;
}

//------------------------------------------------------------------------------
void SkeletonPoseBar::pose_set(const tms_ss_kinect_v2::pose_setting::ConstPtr& msg){
  tc = grasp::PlanBase::instance();
  std::string tagId(object_names[msg->user_id]);
  BodyItemPtr item = NULL;
  if (tc->objTag2Item.find(tagId) != tc->objTag2Item.end())
  {
    item = tc->objTag2Item[tagId];
  }
  if (tc->robTag2Arm.find(tagId) != tc->robTag2Arm.end())
  {
    item = tc->robTag2Arm[tagId]->bodyItemRobot;
  }
  if(!item)
  {
    os << "Error: the tagId is not recorded " << tagId << std::endl;
    return;
  }
  if (item->body()->numJoints() != (int)msg->joint_angle.size())
  {
    os << "Error: the number of Joints of input should be " << item->body()->numJoints()
      << std::endl;
    return;
  }

  //temporary
  if (msg->position[1] < 1.3)
  {
    tc->RemoveEnvironment(item);
    ItemTreeView::mainInstance()->checkItem(item,false);
    return;
  }
  else
  {
    ItemTreeView::mainInstance()->checkItem(item,true);
  }
  for(int i=0;i<3;i++)
  {
    item->body()->link(0)->p()(i) = msg->position[i];
  }
  item->body()->link(0)->R() = rotFromRpy(msg->position[3],msg->position[4],msg->position[5]);
  item->calcForwardKinematics();
  for(int i=0;i<item->body()->numJoints();i++){
    item->body()->joint(i)->q() = msg->joint_angle[i];
  }
  item->calcForwardKinematics();
  item->notifyKinematicStateChange();
  tc->flush();

  return;
}


//------------------------------------------------------------------------------
void SkeletonPoseBar::onSetPoseButtonClicked(){
  ros::init(argc, argv, "skeleton_pose");
  static boost::thread thread_connectROS(boost::bind(&SkeletonPoseBar::connectROS, this));
  return;
}

//------------------------------------------------------------------------------
void SkeletonPoseBar::connectROS()
{
  os <<  "Pose setting start." << endl;
  static ros::Rate loop_rate(20); // 0.05sec
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("skeleton_pose_set", 1,
      &SkeletonPoseBar::pose_set, this);
  ros::spin();
  os <<  "Pose setting end." << endl;
}
