#ifndef EXCADE_WAGON_TRAJECTORY_BAR_H_INCLUDED
#define EXCADE_WAGON_TRAJECTORY_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include "./extplugin/graspPlugin/Grasp/GraspBar.h"

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QCheckBox>
#include <QPushButton>
//#include "exportdef.h"

#include "WagonTrajectoryPlanner.h"
#include "WagonPushPlanner.h"
#include "../libVoronoiPathPlanner/VoronoiPathPlanner.h"

#include "exportdef.h"

using namespace cnoid;

namespace cnoid {

    class MessageView;
	
}
    
    namespace grasp {
		
		class SetMapLimitDialog : public QDialog
		{
			public:
				DoubleSpinBox llimit_x;
				DoubleSpinBox llimit_y;
				DoubleSpinBox llimit_z;
				DoubleSpinBox ulimit_x;
				DoubleSpinBox ulimit_y;
				DoubleSpinBox ulimit_z;
				SetMapLimitDialog() ;
				void okClicked();
		};

        class EXCADE_API WagonTrajectoryBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static WagonTrajectoryBar* instance();

            virtual ~WagonTrajectoryBar();


          protected:


          private:

            WagonTrajectoryBar();

            MessageView& mes;
			std::ostream& os;
	//	  GraspController* gc;
			WagonTrajectoryPlanner* WagontrajectoryPlanner_;
            
            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
            
            std::vector<WagonMotionState>	tempMotionSeq;

            void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
            void onBodyItemDetachedFromRoot();
        void setPlanDOF(std::vector<WagonMotionState>& PlanMotionState);
        void onSetMapLimitButtonClicked();
	    void onWagonTrajectoryPlanButtonClicked();
	    
	    void onResetButtonClicked();
	    void onSetStartMotionStateButtonClicked();
	    void onAddMiddleMotionStateButtonClicked();
	    void onSetEndMotionStateButtonClicked();
	    void onCheckMotionStateButtonClicked();
	    
	    void onWagonGraspButtonClicked();
	    void onWagonReleaseButtonClicked();
	    //~ void onWagonPushPlanButtonClicked();
	  
        };
    }

#endif
