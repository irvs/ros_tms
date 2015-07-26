#ifndef _SKELETON_POSE_SET_BAR_H_INCLUDED
#define _SKELETON_POSE_SET_BAR_H_INCLUDED

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <ros/ros.h>
#include <tms_ss_kinect_v2/pose_setting.h>

//~ #include <cnoid/BodyItem>
#include <cnoid/ToolBar>
//~ #include <cnoid/SignalProxy>
#include <cnoid/MessageView>
#include <Grasp/exportdef.h>

using namespace cnoid;

namespace grasp {
  class SkeletonPoseBar : public cnoid::ToolBar, public boost::signals::trackable {
    public:
      SkeletonPoseBar();
      static SkeletonPoseBar* instance();
      virtual ~SkeletonPoseBar();

      bool isRosInit;
      void pose_set(const tms_ss_kinect_v2::pose_setting::ConstPtr& msg);

    private:
      MessageView& mes;
      std::ostream& os;

      void onSetPoseButtonClicked();
  };
}

#endif
