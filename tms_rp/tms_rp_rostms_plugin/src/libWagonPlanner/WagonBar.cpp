#include "WagonBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include "./extplugin/graspPlugin/Grasp/PlanBase.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int WagonBar::count = 0;

SetWagonParamDialog::SetWagonParamDialog() : QDialog(MainWindow::instance()) {
	
	setWindowTitle("Set Wagon Param");
	
	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);
	
	QHBoxLayout* hbox;
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Long Side Length   "));
	LongLength.setAlignment(Qt::AlignCenter);
	LongLength.setDecimals(3);
	LongLength.setRange(0, 3.000);
	LongLength.setSingleStep(0.001);
	LongLength.setValue(0.435);
	hbox->addWidget(&LongLength);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Short Side Length  "));
	ShortLength.setAlignment(Qt::AlignCenter);
	ShortLength.setDecimals(3);
	ShortLength.setRange(0, 3.000);
	ShortLength.setSingleStep(0.001);
	ShortLength.setValue(0.310);
	hbox->addWidget(&ShortLength);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Height                        "));
	Height.setAlignment(Qt::AlignCenter);
	Height.setDecimals(3);
	Height.setRange(0, 3.000);
	Height.setSingleStep(0.001);
	Height.setValue(1.125);
	hbox->addWidget(&Height);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Control Distance    "));
	ControlDist.setAlignment(Qt::AlignCenter);
	ControlDist.setDecimals(3);
	ControlDist.setRange(0, 3.000);
	ControlDist.setSingleStep(0.001);
	ControlDist.setValue(0.400);
	//~ ControlDist.setValue(0.500);
	//~ ControlDist.setValue(0.550);
	hbox->addWidget(&ControlDist);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Grasp Point Height"));
	graspHeight.setAlignment(Qt::AlignCenter);
	graspHeight.setDecimals(3);
	graspHeight.setRange(0, 3.000);
	graspHeight.setSingleStep(0.001);
	graspHeight.setValue(0.900);
	hbox->addWidget(&graspHeight);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&SetWagonParamDialog::okClicked, this));
	
	vbox->addWidget(okButton);
}

void SetWagonParamDialog::okClicked(){
	WagonPlanner::instance()->targetWagon->size_LongSide_Length = LongLength.value();
	WagonPlanner::instance()->targetWagon->size_ShortSide_Length = ShortLength.value();
	WagonPlanner::instance()->targetWagon->size_Height = Height.value();
	WagonPlanner::instance()->targetWagon->controlDist = ControlDist.value();
	WagonPlanner::instance()->targetWagon->graspHeight = graspHeight.value();
	WagonPlanner::instance()->targetWagon->rel_control_point[0] = ControlDist.value();
	WagonPlanner::instance()->targetWagon->rel_control_point[1] = 0.0;
	WagonPlanner::instance()->targetWagon->rel_control_point[2] = graspHeight.value();
	
	MessageView::mainInstance()->cout() << "Set wagon param"<< endl;
	
	SetWagonGraspOffsetDialog* SWGODialog = new SetWagonGraspOffsetDialog();
	SWGODialog->show();
}

SetWagonGraspOffsetDialog::SetWagonGraspOffsetDialog() : QDialog(MainWindow::instance()) {
	
	setWindowTitle("Set Wagon Grasp PalmPos Offset");
	
	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);
	
	QHBoxLayout* hbox;
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(" x (Length of the palm)		"));
	offset_x.setAlignment(Qt::AlignCenter);
	offset_x.setDecimals(3);
	offset_x.setRange(-1.000, 1.000);
	offset_x.setSingleStep(0.001);
	//~ offset_x.setValue(-0.150);
	offset_x.setValue(-0.160);
	hbox->addWidget(&offset_x);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(" y (Thickness of the palm)		"));
	offset_y.setAlignment(Qt::AlignCenter);
	offset_y.setDecimals(3);
	offset_y.setRange(-1.000, 1.000);
	offset_y.setSingleStep(0.001);
	offset_y.setValue(0.015);
	hbox->addWidget(&offset_y);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(" z (Adjustment of the grasp height)  "));
	offset_z.setAlignment(Qt::AlignCenter);
	offset_z.setDecimals(3);
	offset_z.setRange(-1.000, 1.000);
	offset_z.setSingleStep(0.001);
	offset_z.setValue(0.000);
	hbox->addWidget(&offset_z);
	hbox->addWidget(new QLabel("(m)"));
	hbox->addStretch();
	vbox->addLayout(hbox);
	

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&SetWagonGraspOffsetDialog::okClicked, this));
	
	vbox->addWidget(okButton);
}

void SetWagonGraspOffsetDialog::okClicked(){
	WagonPlanner::instance()->targetWagon->graspPos_offset[0] = offset_x.value();
	WagonPlanner::instance()->targetWagon->graspPos_offset[1] = offset_y.value();
	WagonPlanner::instance()->targetWagon->graspPos_offset[2] = offset_z.value();
	MessageView::mainInstance()->cout() << "Set wagon grasp pos offset"<< endl;
}

WagonBar* WagonBar::instance()
{
	static WagonBar* instance = new WagonBar();
	return instance;
}

WagonBar::WagonBar()
	: ToolBar("WagonBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=WagonPlan="));

	addButton(("SetWagon"), ("Set Wagon"))->
		sigClicked().connect(bind(&WagonBar::onSetWagonButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("SetWagonGraspOffset"), ("Set Wagon Grasp PalmPos Offset"))->
		sigClicked().connect(bind(&WagonBar::onSetWagonGraspOffsetButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("SetWagonSlaveItems"), ("Set Wagon Slave Items"))->
		sigClicked().connect(bind(&WagonBar::onSetWagonSlaveItemsButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("RemoveWagonSlaveItems"), ("Remove Wagon Slave Items"))->
		sigClicked().connect(bind(&WagonBar::onRemoveWagonSlaveItemsButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("updateWagonPos"), ("Update Wagon Pos"))->
		sigClicked().connect(bind(&WagonBar::onupdateWagonPosButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addSeparator();
	
	ItemTreeView::mainInstance()->sigSelectionChanged().connect(
		bind(&WagonBar::onItemSelectionChanged, this, _1));
	count++;
}


WagonBar::~WagonBar()
{
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	count--;
}

void WagonBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
	bool selectedBodyItemsChanged = false;
	if(count < 1) return;
	if(selectedBodyItems_ != bodyItems){
		selectedBodyItems_ = bodyItems;
		selectedBodyItemsChanged = true;
	}

	BodyItemPtr firstItem = bodyItems.toSingle();

	if(firstItem && firstItem != currentBodyItem_){
		currentBodyItem_ = firstItem;
		connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
		connectionOfCurrentBodyItemDetachedFromRoot = currentBodyItem_->sigDetachedFromRoot().connect(
			bind(&WagonBar::onBodyItemDetachedFromRoot, this));
		sigCurrentBodyItemChanged_(currentBodyItem_.get());
	}

	if(selectedBodyItemsChanged){
		sigBodyItemSelectionChanged_(selectedBodyItems_);
	}

	targetBodyItems.clear();
	if(selectedBodyItems_.empty()){
//		if(currentBodyItem_){
//			targetBodyItems.push_back(currentBodyItem_);
//		}
	} else {
		targetBodyItems = selectedBodyItems_;
	}
}

void WagonBar::onBodyItemDetachedFromRoot()
{
	currentBodyItem_ = 0;
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	sigCurrentBodyItemChanged_(0);
}

void WagonBar::onSetWagonButtonClicked()
{
	if(targetBodyItems.size()==1){
		WagonPlanner::instance()->SetWagon(targetBodyItems[0]);
		os << WagonPlanner::instance()->targetWagon->name() << " is wagon"<< endl;
	}else{
		os <<  "Please selecet one bodyitem" << endl;
		return;
	}
	
	SetWagonParamDialog* SWPDialog = new SetWagonParamDialog();
	SWPDialog->show();
	
	WagonPlanner::instance()->initialCollision();
}

void WagonBar::onSetWagonGraspOffsetButtonClicked()
{
	WagonPlanner* wp = WagonPlanner::instance();
	if(!wp->targetWagon){
		os<<"Please Set Wagon!"<<endl;
		return;
	}

	SetWagonGraspOffsetDialog* SWGODialog = new SetWagonGraspOffsetDialog();
	SWGODialog->show();
}

void WagonBar::onSetWagonSlaveItemsButtonClicked()
{
	WagonPlanner* wp = WagonPlanner::instance();
	if(!wp->targetWagon){
		os<<"Please Set Wagon!"<<endl;
		return;
	}
	if(targetBodyItems.size()>0){
		for(unsigned int i=0;i<targetBodyItems.size();i++){
			if(wp->targetWagon->name()!=targetBodyItems[i]->name()){
				wp->SetWagonSlaveItem(targetBodyItems[i]);
				os << targetBodyItems[i]->name() << " is wagon slave item"<< endl;
			}
		}
	}else{
		os <<  "Please selecet bodyitems" << endl;	
	}
	
	WagonPlanner::instance()->initialCollision();
}

void WagonBar::onRemoveWagonSlaveItemsButtonClicked()
{
	WagonPlanner* wp = WagonPlanner::instance();
	if(!wp->targetWagon){
		os<<"Please Set Wagon!"<<endl;
		return;
	}
	if(targetBodyItems.size()>0){
		for(unsigned int i=0;i<targetBodyItems.size();i++){
			wp->RemoveWagonSlaveItem(targetBodyItems[i]);
			os << targetBodyItems[i]->name() << " is removed from wagon slave item"<< endl;
		}
	}else{
		os <<  "Please selecet bodyitems" << endl;	
	}
	
	WagonPlanner::instance()->initialCollision();
}

void WagonBar::onupdateWagonPosButtonClicked()
{
	WagonPlanner* wp = WagonPlanner::instance();
	if(!wp->targetWagon)
		os<<"Please Set Wagon!"<<endl;
	else{
		PlanBase::instance()->calcForwardKinematics();
		PlanBase::instance()->flush();
		//~ wp->flush();
		//~ os<<wp->isColliding()<<endl;
		os<<PlanBase::instance()->isColliding()<<endl;
	}
}
