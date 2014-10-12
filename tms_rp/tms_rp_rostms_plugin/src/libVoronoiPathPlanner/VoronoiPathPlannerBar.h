#ifndef EXCADE_VORONOI_PATH_PLANNER_BAR_H_INCLUDED
#define EXCADE_VORONOI_PATH_PLANNER_BAR_H_INCLUDED

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/SignalProxy>  
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <Grasp/GraspBar.h>

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QCheckBox>
#include <QPushButton>

#include "VoronoiPathPlanner.h"


#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif


using namespace cnoid;

namespace cnoid {
  class MessageView;
}
    
namespace grasp {

  class SetMapParamDialog : public QDialog
  {
  	public:
  		DoubleSpinBox x_llimit;
  		DoubleSpinBox x_ulimit;
  		DoubleSpinBox y_llimit;
  		DoubleSpinBox y_ulimit;
  		DoubleSpinBox cell_size;
  		SetMapParamDialog() ;
  		void okClicked();
  };

  class EXCADE_API VoronoiPathPlannerBar : public cnoid::ToolBar, public boost::signals::trackable
  {
    public:

      static VoronoiPathPlannerBar* instance();

      virtual ~VoronoiPathPlannerBar();

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

      VoronoiPathPlannerBar();

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

      void onSetCollisionTargetButtonClicked();
      void onMakeCollisionMapButtonClicked();
      void onLoadCollisionMapButtonClicked();

      void onResetButtonClicked();
      void onSetStartPosButtonClicked();
      void onSetGoalPosButtonClicked();
      void onStartPlanButtonClicked();

    static int count;
  };
}

#endif
