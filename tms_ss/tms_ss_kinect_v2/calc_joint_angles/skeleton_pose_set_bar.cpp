//~ #include <tms_rp_controller.h>
#include <skeleton_pose_set_bar.h>
#include <tms_ss_kinect_v2/pose_setting.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <cnoid/MessageView>
#include <Grasp/PlanBase.h>
#include <Grasp/VectorMath.h>

int argc;
char **argv;

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

//------------------------------------------------------------------------------
SkeletonPoseBar* SkeletonPoseBar::instance()
{
  static SkeletonPoseBar* instance = new SkeletonPoseBar();
  return instance;
}

//------------------------------------------------------------------------------
SkeletonPoseBar::SkeletonPoseBar(): ToolBar("SkeletonPoseBar"),
                              mes(*MessageView::mainInstance()),
                              os(MessageView::mainInstance()->cout())
{

  // ros init
  //~ if (!isRosInit) {
    //~ ros::init(argc, argv, "skeleton_pose");
    //~ isRosInit=true;
  //~ }

  // ros nodehandle, topic, service init
  //~ static ros::NodeHandle nh;
  //~ ros::ServiceServer service_set = nh.advertiseService("set_skeleton_pose", set_skeleton_pose);

  addSeparator();

  addLabel(("=SkeletonPose="));

  addButton(("Set Pose"), ("Set Skeleton Pose"))->
    sigClicked().connect(bind(&SkeletonPoseBar::onSetPoseButtonClicked, this));

}

//------------------------------------------------------------------------------
SkeletonPoseBar::~SkeletonPoseBar()
{
  return;
}

//------------------------------------------------------------------------------
void SkeletonPoseBar::pose_set(const tms_ss_kinect_v2::pose_setting::ConstPtr& msg){
	PlanBase* tc = PlanBase::instance();
	if(!tc->targetArmFinger){
		cout<<"error: Set Robot"<<endl;
		return;
	}

	for(int i=0;i<3;i++)
		tc->base()->p()(i) = msg->position[i];
	tc->base()->R() = rotFromRpy(msg->position[3],msg->position[4],msg->position[5]);
	tc->calcForwardKinematics();
	for(int i=0;i<tc->body()->numJoints();i++){
		tc->body()->joint(i)->q()=msg->joint_angle[i];
		//~ cout<<i<<":"<<msg->joint_angle[i]<<endl;
	}
	tc->calcForwardKinematics();
	tc->flush();

	return;
}


//------------------------------------------------------------------------------
void SkeletonPoseBar::onSetPoseButtonClicked(){
  ros::init(argc, argv, "skeleton_pose");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("skeleton_pose_set", 1,
      &SkeletonPoseBar::pose_set, this);

  ros::spin();
  return;
}

