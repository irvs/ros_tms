/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_GIVEOBJTOHUMANPLANNER_BAR_H_INCLUDED
#define EXCADE_GIVEOBJTOHUMANPLANNER_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include "./extplugin/graspPlugin/Grasp/GraspBar.h"
//~ #include "../Grasp/GraspController.h"

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QCheckBox>
#include <QPushButton>

#include "HumanPlanner.h"
#include "GiveObjToHumanPlanner.h"

#include "exportdef.h"

#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif


using namespace cnoid;

namespace cnoid {

    class MessageView;
	
}
    
    namespace grasp {
		
        class EXCADE_API GiveObjToHumanPlannerBar : public cnoid::ToolBar, public boost::signals::trackable
        {
          public:

            static GiveObjToHumanPlannerBar* instance();

            virtual ~GiveObjToHumanPlannerBar();
            
            boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
                return sigBodyItemSelectionChanged_;
            }

            cnoid::SignalProxy< boost::signal<void(cnoid::BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
                return sigCurrentBodyItemChanged_;
            }

            const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems() {
                return selectedBodyItems_;
            }

            cnoid::BodyItem* currentBodyItem() {
                return currentBodyItem_.get();
            }

            bool makeSingleSelection(cnoid::BodyItemPtr bodyItem);


          private:

            GiveObjToHumanPlannerBar();

            MessageView& mes;
			std::ostream& os;
			cnoid::BodyItemPtr currentBodyItem_;
            cnoid::ItemList<cnoid::BodyItem> selectedBodyItems_;
            cnoid::ItemList<cnoid::BodyItem> targetBodyItems;
            boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            
            boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
            
			void onItemSelectionChanged(const cnoid::ItemList<cnoid::BodyItem>& bodyItems);
			void onBodyItemDetachedFromRoot();
			
			void onSetHumanButtonClicked();
			void onMakeManipulabilityMapButtonClicked();
			void oncalcGiveObjPosButtonClicked();
			void oncalcGetObjPosButtonClicked();
		
		static int count;
        };
    }

#endif
