#include "VoronoiPathPlannerBar.h"
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView> 
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <Grasp/PlanBase.h>

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int VoronoiPathPlannerBar::count = 0;

SetMapParamDialog::SetMapParamDialog() : QDialog(MainWindow::instance()) {
  
  setWindowTitle("Set Collision Map Param");
  
  QVBoxLayout* vbox = new QVBoxLayout();
  setLayout(vbox);
  
  QHBoxLayout* hbox;
  
  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" x_llimit  "));
  x_llimit.setAlignment(Qt::AlignCenter);
  x_llimit.setDecimals(2);
  x_llimit.setRange(-10.00, 10.00);
  x_llimit.setSingleStep(0.01);
  x_llimit.setValue(0.00);
  hbox->addWidget(&x_llimit);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);
  
  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" x_ulimit  "));
  x_ulimit.setAlignment(Qt::AlignCenter);
  x_ulimit.setDecimals(2);
  x_ulimit.setRange(-10.00, 10.00);
  x_ulimit.setSingleStep(0.01);
  x_ulimit.setValue(8.00);
  hbox->addWidget(&x_ulimit);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);
  
  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" y_llimit  "));
  y_llimit.setAlignment(Qt::AlignCenter);
  y_llimit.setDecimals(2);
  y_llimit.setRange(-10.00, 10.00);
  y_llimit.setSingleStep(0.01);
  y_llimit.setValue(0.00);
  hbox->addWidget(&y_llimit);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);
  
  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" y_ulimit  "));
  y_ulimit.setAlignment(Qt::AlignCenter);
  y_ulimit.setDecimals(2);
  y_ulimit.setRange(-10.00, 10.00);
  y_ulimit.setSingleStep(0.01);
  y_ulimit.setValue(4.50);
  hbox->addWidget(&y_ulimit);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);
  
  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" cell_size  "));
  cell_size.setAlignment(Qt::AlignCenter);
  cell_size.setDecimals(2);
  cell_size.setRange(-10.00, 10.00);
  cell_size.setSingleStep(0.01);
  cell_size.setValue(0.01);
  hbox->addWidget(&cell_size);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);
  
  cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
  okButton->setDefault(true);
  connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
  okButton->sigClicked().connect(boost::bind(&SetMapParamDialog::okClicked, this));
  
  vbox->addWidget(okButton);
}

void SetMapParamDialog::okClicked(){
  VoronoiPathPlanner::instance()->x_llimit = x_llimit.value();
  VoronoiPathPlanner::instance()->x_ulimit = x_ulimit.value();
  VoronoiPathPlanner::instance()->y_llimit = y_llimit.value();
  VoronoiPathPlanner::instance()->y_ulimit = y_ulimit.value();
  VoronoiPathPlanner::instance()->cell_size = cell_size.value();
  MessageView::mainInstance()->cout() << "Set collision map param"<< endl;
}

VoronoiPathPlannerBar* VoronoiPathPlannerBar::instance()
{
  static VoronoiPathPlannerBar* instance = new VoronoiPathPlannerBar();
  return instance;
}

VoronoiPathPlannerBar::VoronoiPathPlannerBar()
  : ToolBar("VoronoiPathPlannerBar"),
    mes(*MessageView::mainInstance()),
     os (MessageView::mainInstance()->cout() )
{
  
//  addSeparator();
  
//  addLabel(("[CollisionMap]"));

//  addButton(("SetCollisionTarget"), ("set collision target model"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onSetCollisionTargetButtonClicked, this));

//  addButton(("makeCollisionMap"), ("make collision map"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onMakeCollisionMapButtonClicked, this));
    
//  addButton(("loadCollisionMap"), ("load collision map"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onLoadCollisionMapButtonClicked, this));
    
//  addSeparator();
  
//  addButton(("Reset"), ("Reset Start & Goal Pos"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onResetButtonClicked, this));
  
//  addButton(("SetStart"), ("Set Start Pos"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onSetStartPosButtonClicked, this));
    
//  addButton(("SetGoal"), ("Set Goal Pos"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onSetGoalPosButtonClicked, this));

//  addButton(("StartPlanning"), ("Start Voronoi Path Planning"))->
//    sigClicked().connect(bind(&VoronoiPathPlannerBar::onStartPlanButtonClicked, this));

  addSeparator();
  
  ItemTreeView::mainInstance()->sigSelectionChanged().connect(
    bind(&VoronoiPathPlannerBar::onItemSelectionChanged, this, _1));
  count++;
}


VoronoiPathPlannerBar::~VoronoiPathPlannerBar()
{
  connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
  count--;
}

void VoronoiPathPlannerBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
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
      bind(&VoronoiPathPlannerBar::onBodyItemDetachedFromRoot, this));
    sigCurrentBodyItemChanged_(currentBodyItem_.get());
  }

  if(selectedBodyItemsChanged){
    sigBodyItemSelectionChanged_(selectedBodyItems_);
  }

  targetBodyItems.clear();
  if(selectedBodyItems_.empty()){
//    if(currentBodyItem_){
//      targetBodyItems.push_back(currentBodyItem_);
//    }
  } else {
    targetBodyItems = selectedBodyItems_;
  }
}

void VoronoiPathPlannerBar::onBodyItemDetachedFromRoot()
{
  currentBodyItem_ = 0;
  connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
  sigCurrentBodyItemChanged_(0);
}

void VoronoiPathPlannerBar::onSetCollisionTargetButtonClicked()
{
  if(targetBodyItems.size()==1){
    VoronoiPathPlanner::instance()->SetCollisionTarget(targetBodyItems[0]);
    os << VoronoiPathPlanner::instance()->collisionTarget->name() << " is collision target"<< endl;
  }else{
    os <<  "Please selecet one bodyitem" << endl;
    return;
  }
}

void VoronoiPathPlannerBar::onMakeCollisionMapButtonClicked()
{
  QString DirName  = QDir::currentPath();
  QString fileName = QFileDialog::getSaveFileName(
  this,//親widget
  tr("Save Collision Map"),// ダイアログタイトル
  DirName,// 保存パス
  tr("CSV Files (*.csv)")
  );
   
  if (fileName.isEmpty())
    return;
  else {
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
      cout<<" can not open file"<<endl;
      return;
    }
    
    SetMapParamDialog* SMPDialog = new SetMapParamDialog();
    SMPDialog->show();
    if(SMPDialog->exec()){}
    
    vector<vector<int> > collision_map;
    VoronoiPathPlanner::instance()->makeCollisionMap(collision_map);
    
    QFile use_file(DirName+"/use_collision_map.csv");
    if (!use_file.open(QIODevice::WriteOnly)) {
      cout<<DirName.toStdString()<<endl;
      cout<<"... can not open file"<<endl;
      return;
    }
    
    char temp[1024];
    sprintf(temp, "%f,%f,%f,%f,%f\n", VoronoiPathPlanner::instance()->x_llimit, VoronoiPathPlanner::instance()->x_ulimit,
								                      VoronoiPathPlanner::instance()->y_llimit, VoronoiPathPlanner::instance()->y_ulimit,
								                      VoronoiPathPlanner::instance()->cell_size);
    file.write(temp);
    use_file.write(temp);
    
    for(int x=0;x<collision_map.size();x++){
      for(int y=0;y<collision_map[x].size();y++){
        sprintf(temp, "%d,", collision_map[x][y] );
        file.write(temp);
        use_file.write(temp);
      }
      file.write("\n");
      use_file.write("\n");
    }
    
    file.close();
    use_file.close();
  }
}

void VoronoiPathPlannerBar::onLoadCollisionMapButtonClicked()
{
  QString DirName = QDir::currentPath();
    QString fileName = QFileDialog::getOpenFileName(
    this,//親widget
    tr("Load Collision Map"),// ダイアログタイトル
    DirName,// 保存パス
    tr("CSV Files (*.csv)")
    );
    
  if (fileName == (DirName+"use_collision_map.csv")){
    os<<"Error : can not load [use_collision_map.csv] "<<endl;
    os<<"Please select other map."<<endl;
    return;
  }
  if (fileName.isEmpty())
    return;
  else {
    VoronoiPathPlanner::instance()->loadCollisionMap(fileName);
  }
}

void VoronoiPathPlannerBar::onResetButtonClicked(){
  VoronoiPathPlanner::instance()->start_pos.clear();
  VoronoiPathPlanner::instance()->start_pos.resize(3);
  VoronoiPathPlanner::instance()->goal_pos.clear();
  VoronoiPathPlanner::instance()->goal_pos.resize(3);
}

void VoronoiPathPlannerBar::onSetStartPosButtonClicked(){
  VoronoiPathPlanner::instance()->start_pos.resize(3);
  VoronoiPathPlanner::instance()->start_pos[0] = PlanBase::instance()->body()->link(0)->p()(0);
  VoronoiPathPlanner::instance()->start_pos[1] = PlanBase::instance()->body()->link(0)->p()(1);
  VoronoiPathPlanner::instance()->start_pos[2] = rpyFromRot(PlanBase::instance()->body()->link(0)->R())(2);
  os<<"Set Start Pos : x = "<<VoronoiPathPlanner::instance()->start_pos[0]<<" y = "<<VoronoiPathPlanner::instance()->start_pos[1]<<" th = "<<rad2deg(VoronoiPathPlanner::instance()->start_pos[2])<<endl;
}

void VoronoiPathPlannerBar::onSetGoalPosButtonClicked(){
  VoronoiPathPlanner::instance()->goal_pos.resize(3);
  VoronoiPathPlanner::instance()->goal_pos[0] = PlanBase::instance()->body()->link(0)->p()(0);
  VoronoiPathPlanner::instance()->goal_pos[1] = PlanBase::instance()->body()->link(0)->p()(1);
  VoronoiPathPlanner::instance()->goal_pos[2] = rpyFromRot(PlanBase::instance()->body()->link(0)->R())(2);
  os<<"Set Goal Pos : x = "<<VoronoiPathPlanner::instance()->goal_pos[0]<<" y = "<<VoronoiPathPlanner::instance()->goal_pos[1]<<" th = "<<rad2deg(VoronoiPathPlanner::instance()->goal_pos[2])<<endl;
}

void VoronoiPathPlannerBar::onStartPlanButtonClicked()
{  
  vector<vector<double> > plan_path;
    VoronoiPathPlanner::instance()->planVoronoiPath(VoronoiPathPlanner::instance()->collisionMap, VoronoiPathPlanner::instance()->start_pos, VoronoiPathPlanner::instance()->goal_pos, plan_path);
    
    for(int i=0;i<plan_path.size();i++){
    PlanBase::instance()->body()->link(0)->p()(0) = plan_path[i][0];
    PlanBase::instance()->body()->link(0)->p()(1) = plan_path[i][1];
    PlanBase::instance()->body()->link(0)->R() = rotFromRpy(0.0,0.0,plan_path[i][2]);
    PlanBase::instance()->calcForwardKinematics();
    PlanBase::instance()->flush();
  }
}
