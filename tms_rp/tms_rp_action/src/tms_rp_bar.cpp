#include <tms_rp_bar.h>
#include <tms_rp_controller.h>
#include <tms_rp_static_map.h>
#include <tms_rp_collision_map.h>
#include <sg_points_get.h>
#include <draw_points.h>
#include <tms_rp_pp.h>

//------------------------------------------------------------------------------
#define MAX_ICS_OBJECT_NUM    25
#define MAX_FURNITURE_NUM     20

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

//------------------------------------------------------------------------------
std::string TmsRpBar::object_name_[25] = {"chipstar_red","chipstar_orange",
                                        "chipstar_green","greentea_bottle",
                                        "soukentea_bottle","cancoffee",
                                        "seasoner_bottle","dispenser",
                                        "soysauce_bottle_black","soysauce_bottle_blue",
                                        "soysauce_bottle_white","pepper_bottle_black",
                                        "pepper_bottle_red","sake_bottle",
                                        "teapot","chawan","teacup1","teacup2","cup1",
                                        "cup2","mugcup","remote","book_red","book_blue","dish"};

std::string TmsRpBar::furniture_name_[20] = {"big_sofa","mini_sofa","small_table","tv_table","tv",
                                           "partition1","partition2","partition3","bed","shelf",
                                           "big_shelf","desk","chair_desk","table","chair_table1","chair_table2",
                                           "shelfdoor","shelf2","wagon","sidetable"};

// initialize static variables
bool TmsRpBar::is_ros_Init_ = false;
bool TmsRpBar::production_version_ = false;
int  TmsRpBar::planning_mode_ = 0; // view mode
int  TmsRpBar::grasping_ = 0;

//------------------------------------------------------------------------------
SetMapParamDialog::SetMapParamDialog() : QDialog(MainWindow::instance()) {
  setWindowTitle("Set Collision Map Param");

  QVBoxLayout* vbox = new QVBoxLayout();
  QHBoxLayout* hbox;

  setLayout(vbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" x_llimit_  "));
  x_llimit_.setAlignment(Qt::AlignCenter);
  x_llimit_.setDecimals(2);
  x_llimit_.setRange(-10.00, 10.00);
  x_llimit_.setSingleStep(0.01);
  x_llimit_.setValue(0.00);
  hbox->addWidget(&x_llimit_);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" x_ulimit_  "));
  x_ulimit_.setAlignment(Qt::AlignCenter);
  x_ulimit_.setDecimals(2);
  x_ulimit_.setRange(-10.00, 10.00);
  x_ulimit_.setSingleStep(0.01);
  x_ulimit_.setValue(8.00);
  hbox->addWidget(&x_ulimit_);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" y_llimit_  "));
  y_llimit_.setAlignment(Qt::AlignCenter);
  y_llimit_.setDecimals(2);
  y_llimit_.setRange(-10.00, 10.00);
  y_llimit_.setSingleStep(0.01);
  y_llimit_.setValue(0.00);
  hbox->addWidget(&y_llimit_);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" y_ulimit_  "));
  y_ulimit_.setAlignment(Qt::AlignCenter);
  y_ulimit_.setDecimals(2);
  y_ulimit_.setRange(-10.00, 10.00);
  y_ulimit_.setSingleStep(0.01);
  y_ulimit_.setValue(4.50);
  hbox->addWidget(&y_ulimit_);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" cell_size_  "));
  cell_size_.setAlignment(Qt::AlignCenter);
  cell_size_.setDecimals(2);
  cell_size_.setRange(-10.00, 10.00);
  cell_size_.setSingleStep(0.01);
  cell_size_.setValue(0.10);
  hbox->addWidget(&cell_size_);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);

  cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
  okButton->setDefault(true);
  connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
  okButton->sigClicked().connect(boost::bind(&SetMapParamDialog::okClicked, this));

  vbox->addWidget(okButton);
}

//------------------------------------------------------------------------------
void SetMapParamDialog::okClicked() {
  TmsRpCollisionMap::instance()->x_llimit_ = x_llimit_.value();
  TmsRpCollisionMap::instance()->x_ulimit_ = x_ulimit_.value();
  TmsRpCollisionMap::instance()->y_llimit_ = y_llimit_.value();
  TmsRpCollisionMap::instance()->y_ulimit_ = y_ulimit_.value();
  TmsRpCollisionMap::instance()->cell_size_ = cell_size_.value();
  MessageView::mainInstance()->cout() << "Set collision map param"<< endl;
}

//------------------------------------------------------------------------------
SelectGoalPosDialog::SelectGoalPosDialog() : QDialog(cnoid::MainWindow::instance()) {
  setWindowTitle("Input Goal Position");

  QVBoxLayout* vbox = new QVBoxLayout();
  QHBoxLayout* hbox = new QHBoxLayout();

  setLayout(vbox);

  hbox->addWidget(new QLabel("x:"));
  goal_pos_x_.setAlignment(Qt::AlignCenter);
  goal_pos_x_.setRange(0, 8000);
  goal_pos_x_.setValue(0);
  hbox->addWidget(&goal_pos_x_);
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel("y:"));
  goal_pos_y_.setAlignment(Qt::AlignCenter);
  goal_pos_y_.setRange(0, 4500);
  goal_pos_y_.setValue(0);
  hbox->addWidget(&goal_pos_y_);
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel("ry:"));
  goal_pos_ry_.setAlignment(Qt::AlignCenter);
  goal_pos_ry_.setRange(-360, 360);
  goal_pos_ry_.setValue(0);
  hbox->addWidget(&goal_pos_ry_);
  hbox->addStretch();
  vbox->addLayout(hbox);

  cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
  okButton->setDefault(true);
  connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
  okButton->sigClicked().connect(boost::bind(&SelectGoalPosDialog::okClicked, this));

  vbox->addWidget(okButton);
}

//------------------------------------------------------------------------------
void SelectGoalPosDialog::okClicked() {
  MessageView::mainInstance()->cout() << "input" << goal_pos_x_.value() << " " << goal_pos_y_.value() << " " << goal_pos_ry_.value() << endl;
  TmsRpBar::instance()->goal_position_x_  = goal_pos_x_.value();
  TmsRpBar::instance()->goal_position_y_  = goal_pos_y_.value();
  TmsRpBar::instance()->goal_position_ry_ = goal_pos_ry_.value();
}

//------------------------------------------------------------------------------
TmsRpBar* TmsRpBar::instance() {
  static TmsRpBar* instance = new TmsRpBar();
  return instance;
}

//------------------------------------------------------------------------------
TmsRpBar::TmsRpBar(): ToolBar("TmsRpBar"), mes_(*MessageView::mainInstance()),
                      os_(MessageView::mainInstance()->cout()),
                      tac_(*TmsRpController::instance()),
                      argc_(), argv_() {
  // initialize ros connect
  try{
    if (!is_ros_Init_) {
      ros::init(argc_, argv_, "tms_rp");
      is_ros_Init_=true;
      cout << "Success: connecting roscore.." << endl;
    }
  }
  catch(...) {
    cout << "Error: ros init" << endl;
  }

  sid_ = 100000;

  // ros nodehandle, topic, service init
  static ros::NodeHandle nh;

  get_data_client_       = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  sp5_control_client_    = nh.serviceClient<tms_msg_rc::rc_robot_control>("sp5_control");
  path_planning_client_  = nh.serviceClient<tms_msg_rp::rps_path_planning>("rps_path_planning");
  ardrone_client_        = nh.serviceClient<tms_msg_rc::robot_control>("robot_control");
  subscribe_pcd_         = nh.subscribe("velodyne_points", 10, &TmsRpBar::receivePointCloudData, this);
  subscribe_static_map_ = nh.subscribe("rps_map_data", 10,    &TmsRpBar::receiveStaticMapData, this);
  subscribe_path_map_   = nh.subscribe("rps_robot_path", 10,  &TmsRpBar::receivePathMapData, this);

  //----------------------------------------------------------------------------
  // create person model
  tac_.createRecord(1001,"person_1");

  // create robot model
  tac_.createRecord(2001,"smartpal4");
  tac_.createRecord(2002,"smartpal5_1");
  tac_.createRecord(2003,"smartpal5_2");
  //tac_.createRecord(2004,"turtlebot2");
  tac_.createRecord(2005,"kobuki");
  tac_.createRecord(2006,"kxp");
  tac_.createRecord(2007,"wheelchair");
  tac_.createRecord(2008,"ardrone");
  tac_.createRecord(2009,"refrigerator");
  tac_.createRecord(2010,"refrigerator_open");
  tac_.createRecord(2011,"kxp2");

  // create sensor model
  tac_.createRecord(3011,"portable_sensor_1");
  tac_.createRecord(3012,"portable_sensor_2");
  tac_.createRecord(3013,"portable_sensor_3");
  tac_.createRecord(3014,"portable_sensor_4");
  tac_.createRecord(3015,"portable_sensor_5");

  // create space model
  tac_.createRecord(5001,"floor928");
  tac_.createRecord(5002,"wall928");
  tac_.createRecord(5005,"corridor928");

  // create furniture model
  tac_.createRecord(6001,"big_sofa");
  tac_.createRecord(6002,"mini_sofa");
  tac_.createRecord(6003,"small_table");
  tac_.createRecord(6004,"tv_table");
  tac_.createRecord(6005,"tv");
  tac_.createRecord(6006,"partition1");
  tac_.createRecord(6007,"partition2");
  tac_.createRecord(6008,"partition3");
  tac_.createRecord(6009,"bed");
  tac_.createRecord(6010,"shelf");
  tac_.createRecord(6011,"big_shelf");
  tac_.createRecord(6012,"desk");
  tac_.createRecord(6013,"chair_desk");
  tac_.createRecord(6014,"table");
  tac_.createRecord(6015,"chair_table1");
  tac_.createRecord(6016,"chair_table2");
  tac_.createRecord(6017,"shelfdoor");
  tac_.createRecord(6018,"shelf2");
  tac_.createRecord(6019,"wagon");
  tac_.createRecord(6020,"sidetable");

  // create etc model
  tac_.createRecord(20001,"blink_arrow");
  tac_.createRecord(20002,"person_marker1");
  tac_.createRecord(20003,"person_marker2");
  tac_.createRecord(20004,"person_marker3");
  tac_.createRecord(20005,"person_marker4");
  tac_.createRecord(20006,"person_marker5");
  tac_.createRecord(20007,"ardrone_goal_position");
  tac_.createRecord(20008,"static_map");
  tac_.createRecord(20009,"dynamic_map");
  tac_.createRecord(20010,"local_map");
  tac_.createRecord(20011,"path_map");
  tac_.createRecord(20012,"robot_marker");

  // arrange model
  mat0_       <<  1, 0, 0, 0, 1, 0, 0, 0, 1;  //   0
  mat_ccw90_  <<  0,-1, 0, 1, 0, 0, 0, 0, 1;  //  90
  mat_ccw180_ << -1, 0, 0, 0,-1, 0, 0, 0, 1;  // 180
  mat_cw90_   <<  0, 1, 0,-1, 0, 0, 0, 0, 1;  // -90

  //----------------------------------------------------------------------------
  // appear and setpos models
  tms_msg_db::TmsdbGetData get_db_data;
  double posX, posY, posZ, posRR, posRP, posRY;

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 1001 + sid_;
  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("person_1");
    tac_.setPos("person_1",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("person_1");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
//  get_db_data.request.tmsdb.id = 2001;
//  get_db_data.request.tmsdb.state = 1;
//  if (get_data_client_.call(get_db_data)){
//    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
//    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
//    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
//    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
//    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
//    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

//    tac_.appear("smartpal4");
//    tac_.setPos("smartpal4",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
//  } else {
//    tac_.appear("smartpal4");
//    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
//  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2002 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("smartpal5_1");
    tac_.setPos("smartpal5_1",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("smartpal5_1");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2003 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("smartpal5_2");
    tac_.setPos("smartpal5_2",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("smartpal5_2");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2005 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("kobuki");
    tac_.setPos("kobuki",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("kobuki");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2006 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("kxp");
    tac_.setPos("kxp",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    tac_.disappear("kxp2");
    tac_.setPos("kxp2",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("kxp");
    tac_.disappear("kxp2");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2007 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("wheelchair");
    tac_.setPos("wheelchair",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("wheelchair");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2008 + sid_;

  if (get_data_client_.call(get_db_data)) {
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("ardrone");
    tac_.setPos("ardrone",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    tac_.disappear("ardrone_goal_position");
    tac_.setPos("ardrone_goal_position",   Vector3(posX+1,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("ardrone");
    tac_.disappear("ardrone_goal_position");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2009 + sid_;

  if (get_data_client_.call(get_db_data)){
  posX = (get_db_data.response.tmsdb[0].x)/1000;
    posY = (get_db_data.response.tmsdb[0].y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("refrigerator");
    tac_.setPos("refrigerator",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    tac_.disappear("refrigerator_open");
    tac_.setPos("refrigerator_open",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("refrigerator");
    tac_.disappear("refrigerator_open");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 5001 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z-get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("floor928");
    tac_.setPos("floor928",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("floor928");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 5002 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("wall928");
    tac_.setPos("wall928", Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("wall928");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 5005 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    tac_.appear("corridor928");
    tac_.setPos("corridor928", Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    tac_.appear("corridor928");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  //  appear and setpos furniture model
  int32_t furnitureID = 0;
  for(int32_t i=0; i < MAX_FURNITURE_NUM; i++) {
    furnitureID = i;
    get_db_data.request.tmsdb.id    = 6001 + i + sid_;

    if (get_data_client_.call(get_db_data)) {
      os_ << "[TmsAction] Get info of furniture ID: " << get_db_data.request.tmsdb.id <<"  OK" << endl;

      posX = (get_db_data.response.tmsdb[0].x)/1000;
      posY = (get_db_data.response.tmsdb[0].y)/1000;
      posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
      posRR= deg2rad(get_db_data.response.tmsdb[0].rr);
      posRP= deg2rad(get_db_data.response.tmsdb[0].rp);
      posRY= deg2rad(get_db_data.response.tmsdb[0].ry);
      tac_.appear(furniture_name_[furnitureID]);
      tac_.setPos(furniture_name_[furnitureID],Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    } else {
      tac_.appear(furniture_name_[furnitureID]);
      os_ << "[TmsAction] Failed to call service get_db_data ID: " << get_db_data.request.tmsdb.id << endl;
    }
  }

  //----------------------------------------------------------------------------
  //  appear and setpos object model
  for(int i=0; i<25; i++) {
    tac_.createRecord(7001+i, object_name_[i]);
  }

  //----------------------------------------------------------------------------
  tac_.createRobotRecord(2002,"smartpal5_1");
  //tac_.createRobotRecord(2006,"kxp");

  //------------------------------------------------------------------------------
  addLabel(("[ROS-TMS]"));

  addButton(QIcon(":/action/icons/simulation.png"), ("simulation mode"))
    ->sigClicked().connect(bind(&TmsRpBar::simulationButtonClicked, this));

  addButton(QIcon(":/action/icons/ros.png"), ("connect to the ros"))->
    sigClicked().connect(bind(&TmsRpBar::connectRosButtonClicked, this));

  addSeparator();

  addButton(QIcon(":/action/icons/collision_target.png"), ("set collision target model"))->
    sigClicked().connect(bind(&TmsRpBar::setCollisionTargetButtonClicked, this));

  addButton(QIcon(":/action/icons/collision_map.png"), ("make collision map"))->
    sigClicked().connect(bind(&TmsRpBar::makeCollisionMapButtonClicked, this));

  addSeparator();

  static_map_toggle_ = addToggleButton(QIcon(":/action/icons/static_map.png"), ("staic map"));
  static_map_toggle_->sigToggled().connect(bind(&TmsRpBar::staticMapButtonClicked, this));
  static_map_toggle_->setChecked(false);

  dynamic_map_toggle_= addToggleButton(QIcon(":/action/icons/dynamic_map.png"), ("dynamic map"));
  dynamic_map_toggle_->sigToggled().connect(bind(&TmsRpBar::staticMapButtonClicked, this));
  dynamic_map_toggle_->setChecked(false);

  local_map_toggle_ = addToggleButton(QIcon(":/action/icons/local_map.png"), ("local map"));
  local_map_toggle_->sigToggled().connect(bind(&TmsRpBar::staticMapButtonClicked, this));
  local_map_toggle_->setChecked(false);

  path_map_toggle_ = addToggleButton(QIcon(":/action/icons/path_map.png"), ("option of path view"));
  path_map_toggle_->sigToggled().connect(bind(&TmsRpBar::pathMapButtonClicked, this));
  path_map_toggle_->setChecked(false);

  robot_map_toggle_ = addToggleButton(QIcon(":/action/icons/robot_map.png"), ("option of robot marker"));
  robot_map_toggle_->sigToggled().connect(bind(&TmsRpBar::robotMapButtonClicked, this));
  robot_map_toggle_->setChecked(false);

  addSeparator();

  addButton(QIcon(":/action/icons/drone.png"), ("drone"))->
    sigClicked().connect(bind(&TmsRpBar::ardroneButtonClicked, this));

  addButton(QIcon(":/action/icons/wheelchair.png"), ("wheelchair"))->
    sigClicked().connect(bind(&TmsRpBar::ardroneButtonClicked, this));

  addButton(QIcon(":/action/icons/smartpal.png"), ("smartpal"))->
    sigClicked().connect(bind(&TmsRpBar::ardroneButtonClicked, this));

  ItemTreeView::mainInstance()->sigSelectionChanged().connect(
    bind(&TmsRpBar::itemSelectionChanged, this, _1));
}

//------------------------------------------------------------------------------
TmsRpBar::~TmsRpBar()
{
}

//------------------------------------------------------------------------------
void TmsRpBar::itemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
  selectedBodyItems_ = bodyItems;
  cout << "selectedBodyItems_ size = " << selectedBodyItems_.size() << endl;
  target_body_items_ = selectedBodyItems_;
}

//------------------------------------------------------------------------------
void TmsRpBar::receivePointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*msg, cloud);
  point_cloud_data_ = cloud;
}

//------------------------------------------------------------------------------
void TmsRpBar::receiveStaticMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg)
{
  static_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::receivePathMapData(const tms_msg_rp::rps_route::ConstPtr& msg)
{
  path_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::getPcdData(){

}

//------------------------------------------------------------------------------
void TmsRpBar::setCollisionTargetButtonClicked()
{
  if(target_body_items_.size()==1){
    cout << "target_body_items_ size = " << target_body_items_.size() << endl;
    TmsRpCollisionMap::instance()->SetCollisionTarget(target_body_items_[0]);
    cout << TmsRpCollisionMap::instance()->collisionTarget->name() << " is collision target"<< endl;
  }else{
    cout <<  "Please selecet one bodyitem" << endl;
    return;
  }
}

//------------------------------------------------------------------------------
void TmsRpBar::makeCollisionMapButtonClicked()
{
  QString DirName  = QDir::currentPath();
  QString fileName = QFileDialog::getSaveFileName(this,tr("Save Collision Map"),
                                                  DirName,tr("CSV Files (*.csv)")
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
    TmsRpCollisionMap::instance()->makeCollisionMap(collision_map);

    QFile use_file(DirName+"/use_collision_map.csv");
    if (!use_file.open(QIODevice::WriteOnly)) {
      cout<<DirName.toStdString()<<endl;
      cout<<"... can not open file"<<endl;
      return;
    }

    char temp[1024];
    sprintf(temp, "%f,%f,%f,%f,%f\n", TmsRpCollisionMap::instance()->x_llimit_, TmsRpCollisionMap::instance()->x_ulimit_,
                                                                                      TmsRpCollisionMap::instance()->y_llimit_, TmsRpCollisionMap::instance()->y_ulimit_,
                                                                                      TmsRpCollisionMap::instance()->cell_size_);
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

//------------------------------------------------------------------------------
void TmsRpBar::staticMapButtonClicked() {
  TmsRpController trc;

  if(static_map_data_.rps_map_x.size()==0){
    ROS_INFO("nothing the static map data");
    return;
  }

  if(!static_map_toggle_->isChecked()){
    trc.disappear("static_map");
    return;
  }

  ROS_INFO("On viewer for static map");

  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node  = (SgGroup*)trc.objTag2Item()["static_map"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0){
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&static_map_data_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  ItemTreeView::mainInstance()->checkItem(trc.objTag2Item()["static_map"],false);
  MessageView::mainInstance()->flush();
  ItemTreeView::mainInstance()->checkItem(trc.objTag2Item()["static_map"],true);
  MessageView::mainInstance()->flush();
}

//------------------------------------------------------------------------------
void TmsRpBar::pathMapButtonClicked() {
  TmsRpController trc;

  if(path_map_data_.rps_route.size()==0){
    ROS_INFO("nothing the path map data");
    return;
  }

  if(!path_map_toggle_->isChecked()){
    trc.disappear("path_map");
    return;
  }

  ROS_INFO("On viewer for path map");
  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node  = (SgGroup*)trc.objTag2Item()["path_map"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0){
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&path_map_data_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  ItemTreeView::mainInstance()->checkItem(trc.objTag2Item()["path_map"],false);
  MessageView::mainInstance()->flush();
  ItemTreeView::mainInstance()->checkItem(trc.objTag2Item()["path_map"],true);
  MessageView::mainInstance()->flush();
}

//------------------------------------------------------------------------------
void TmsRpBar::robotMapButtonClicked() {
  TmsRpController trc;

  if(path_map_data_.rps_route.size()==0){
    ROS_INFO("nothing the path map data");
    return;
  }

  if(!robot_map_toggle_->isChecked()){
    trc.disappear("robot_marker");
    return;
  }

  ROS_INFO("On viewer for robot marker");
  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node  = (SgGroup*)trc.objTag2Item()["robot_marker"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0){
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&path_map_data_,true);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  ItemTreeView::mainInstance()->checkItem(trc.objTag2Item()["robot_marker"],false);
  MessageView::mainInstance()->flush();
  ItemTreeView::mainInstance()->checkItem(trc.objTag2Item()["robot_marker"],true);
  MessageView::mainInstance()->flush();
}

//------------------------------------------------------------------------------
void TmsRpBar::updateObjectInfo()
{
  PlanBase *pb = PlanBase::instance();

  if (!pb->robTag2Arm.size()) {
    os_ <<  "[TmsAction] set robot before updateinfo!" << endl;
    return;
  }

  os_ << "[TmsAction] Update TMS DB Information" << endl;

  //----------------------------------------------------------------------------
  // update information of robot in floor
  tms_msg_db::TmsdbGetData getRobotData;

  for(int32_t i=0; i<3; i++) {
      getRobotData.request.tmsdb.id =2001 + i;
      getRobotData.request.tmsdb.sensor = 3001;

    if (get_data_client_.call(getRobotData)) {
      os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
    } else {
      os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
      return;
    }

    if (getRobotData.response.tmsdb.empty()==true) {
      os_ << "[TmsAction] nothing on floor (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
      return;
    }

    if (getRobotData.response.tmsdb[0].state==1) {
      double rPosX = getRobotData.response.tmsdb[0].x/1000;
      double rPosY = getRobotData.response.tmsdb[0].y/1000;
      double rPosZ = 0.0;
      Vector3 rpy (deg2rad(getRobotData.response.tmsdb[0].rp),deg2rad(getRobotData.response.tmsdb[0].rr),deg2rad(getRobotData.response.tmsdb[0].ry));
      Matrix3 rot = rotFromRpy(rpy);

      if(getRobotData.request.tmsdb.id == 2001) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callLater(bind(&TmsRpController::disappear,tac_,"smartpal4"));
        }
        else{
          callLater(bind(&TmsRpController::appear,tac_,"smartpal4"));
          callLater(bind(&TmsRpController::setPos,tac_,"smartpal4",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if(getRobotData.request.tmsdb.id == 2002) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callLater(bind(&TmsRpController::disappear,tac_,"smartpal5_1"));
        }
        else{
          callLater(bind(&TmsRpController::appear,tac_,"smartpal5_1"));
          callLater(bind(&TmsRpController::setPos,tac_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if(getRobotData.request.tmsdb.id == 2003) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callLater(bind(&TmsRpController::disappear,tac_,"smartpal5_2"));
        }
        else{
          callLater(bind(&TmsRpController::appear,tac_,"smartpal5_2"));
          callLater(bind(&TmsRpController::setPos,tac_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
    }
  }

  //----------------------------------------------------------------------------
  // update information of kobuki
  getRobotData.request.tmsdb.id = 2005;
  getRobotData.request.tmsdb.sensor = 3001;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    callLater(bind(&TmsRpController::disappear,tac_,"kobuki"));
  } else if (getRobotData.response.tmsdb[0].state==1) {
    double rPosX = getRobotData.response.tmsdb[0].x/1000;
    double rPosY = getRobotData.response.tmsdb[0].y/1000;
    double rPosZ = getRobotData.response.tmsdb[0].z/1000;
    Vector3 rpy (deg2rad(getRobotData.response.tmsdb[0].rr),deg2rad(getRobotData.response.tmsdb[0].rp),deg2rad(getRobotData.response.tmsdb[0].ry));
    Matrix3 rot = rotFromRpy(rpy);

    if(rPosX == 0.0 && rPosY == 0.0) {
      callLater(bind(&TmsRpController::disappear,tac_,"kobuki"));
    }
    else{
      callLater(bind(&TmsRpController::appear,tac_,"kobuki"));
      callLater(bind(&TmsRpController::setPos,tac_,"kobuki",Vector3(rPosX,rPosY,rPosZ), rot));
    }
  }

  //----------------------------------------------------------------------------
  // update information of kxp
  getRobotData.request.tmsdb.id = 2006;
  getRobotData.request.tmsdb.sensor = 3001;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    callLater(bind(&TmsRpController::disappear,tac_,"kxp"));
  } else if (getRobotData.response.tmsdb[0].state==1) {
    double rPosX = getRobotData.response.tmsdb[0].x/1000;
    double rPosY = getRobotData.response.tmsdb[0].y/1000;
    double rPosZ = getRobotData.response.tmsdb[0].z/1000;
    Vector3 rpy (deg2rad(getRobotData.response.tmsdb[0].rr),deg2rad(getRobotData.response.tmsdb[0].rp),deg2rad(getRobotData.response.tmsdb[0].ry));
    Matrix3 rot = rotFromRpy(rpy);

    if(rPosX == 0.0 && rPosY == 0.0) {
      callLater(bind(&TmsRpController::disappear,tac_,"kxp"));
    }
    else{
      callLater(bind(&TmsRpController::appear,tac_,"kxp"));
      callLater(bind(&TmsRpController::setPos,tac_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot));
    }
  }

  //----------------------------------------------------------------------------
  // update information of wheelchair
  getRobotData.request.tmsdb.id = 2007;
  getRobotData.request.tmsdb.sensor = 3001;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    callLater(bind(&TmsRpController::disappear,tac_,"wheelchair"));
  } else if (getRobotData.response.tmsdb[0].state==1) {
    double rPosX = getRobotData.response.tmsdb[0].x/1000;
    double rPosY = getRobotData.response.tmsdb[0].y/1000;
    double rPosZ = getRobotData.response.tmsdb[0].z/1000;
    Vector3 rpy (deg2rad(getRobotData.response.tmsdb[0].rr),deg2rad(getRobotData.response.tmsdb[0].rp),deg2rad(getRobotData.response.tmsdb[0].ry));
    Matrix3 rot = rotFromRpy(rpy);

    if(rPosX == 0.0 && rPosY == 0.0) {
      callLater(bind(&TmsRpController::disappear,tac_,"wheelchair"));
    }
    else{
      callLater(bind(&TmsRpController::appear,tac_,"wheelchair"));
      callLater(bind(&TmsRpController::setPos,tac_,"wheelchair",Vector3(rPosX,rPosY,rPosZ), rot));
    }
  }

  //----------------------------------------------------------------------------
  // update information of ardrone
  getRobotData.request.tmsdb.id = 2008;
  getRobotData.request.tmsdb.sensor = 3001;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    callLater(bind(&TmsRpController::disappear,tac_,"ardrone"));
  } else if (getRobotData.response.tmsdb[0].state==1) {
    double rPosX = getRobotData.response.tmsdb[0].x/1000;
    double rPosY = getRobotData.response.tmsdb[0].y/1000;
    double rPosZ = getRobotData.response.tmsdb[0].z/1000;
    Vector3 rpy (deg2rad(getRobotData.response.tmsdb[0].rr),deg2rad(getRobotData.response.tmsdb[0].rp),deg2rad(getRobotData.response.tmsdb[0].ry));
    Matrix3 rot = rotFromRpy(rpy);

    if(rPosX == 0.0 && rPosY == 0.0) {
      callLater(bind(&TmsRpController::disappear,tac_,"ardrone"));
    }
    else{
      callLater(bind(&TmsRpController::appear,tac_,"ardrone"));
      callLater(bind(&TmsRpController::setPos,tac_,"ardrone",Vector3(rPosX,rPosY,rPosZ), rot));
    }
  }

  //----------------------------------------------------------------------------
  // update information of refrigerator
  getRobotData.request.tmsdb.id = 2009;
  getRobotData.request.tmsdb.sensor = 2009;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    return;
  }

  if (getRobotData.response.tmsdb[0].state==0) {
    callLater(bind(&TmsRpController::disappear,tac_,"refrigerator"));
    callLater(bind(&TmsRpController::disappear,tac_,"refrigerator_open"));
  } else if (getRobotData.response.tmsdb[0].state==1) {
    callLater(bind(&TmsRpController::appear,tac_,"refrigerator"));
    callLater(bind(&TmsRpController::disappear,tac_,"refrigerator_open"));
  } else if (getRobotData.response.tmsdb[0].state==2) {
    callLater(bind(&TmsRpController::disappear,tac_,"refrigerator"));
    callLater(bind(&TmsRpController::appear,tac_,"refrigerator_open"));
  }

  //----------------------------------------------------------------------------
  // update information of wagon
  getRobotData.request.tmsdb.id = 6019;
  getRobotData.request.tmsdb.sensor = 3001;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    callLater(bind(&TmsRpController::disappear,tac_,"wagon"));
  } else if (getRobotData.response.tmsdb[0].state==1) {
    double rPosX = getRobotData.response.tmsdb[0].x/1000;
    double rPosY = getRobotData.response.tmsdb[0].y/1000;
    double rPosZ = 0.35;
    Vector3 rpy (deg2rad(getRobotData.response.tmsdb[0].rr),deg2rad(getRobotData.response.tmsdb[0].rp),deg2rad(getRobotData.response.tmsdb[0].ry));
    Matrix3 rot = rotFromRpy(rpy);

    if(rPosX == 0.0 && rPosY == 0.0) {
      callLater(bind(&TmsRpController::disappear,tac_,"wagon"));
    }
    else{
      callLater(bind(&TmsRpController::appear,tac_,"wagon"));
      callLater(bind(&TmsRpController::setPos,tac_,"wagon",Vector3(rPosX,rPosY,rPosZ), rot));
    }
  }

  //----------------------------------------------------------------------------
  // update information of objects in shelf
  tms_msg_db::TmsdbGetData getObjectData;

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) object_state_[i]=false;

//  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) {
//    getObjectData.request.tmsdb.id =7001 + i;
//    getObjectData.request.tmsdb.sensor = 3002; // ics

//    if (get_data_client_.call(getObjectData)) {
//      os_ << "[TmsAction] Get info of object ID: " << getObjectData.request.tmsdb.id <<"  OK" << endl;
//    } else {
//      os_ << "[TmsAction] Failed to call service getObjectData ID: " << getObjectData.request.tmsdb.id << endl;
//      return;
//    }

//    if (getObjectData.response.tmsdb.empty()==true) {
//      os_ << "[TmsAction] nothing in shelf" <<endl;
//      return;
//    }

//    if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6010) {
//      int    oID   = getObjectData.response.tmsdb[0].id - 7001;
//      double oPosX = 3.66 + getObjectData.response.tmsdb[0].y/1000;
//      double oPosY = 1.87 - getObjectData.response.tmsdb[0].x/1000;
//      double oPosZ = 0.0 + getObjectData.response.tmsdb[0].z/1000;
//      callLater(bind(&TmsRpController::appear,tac_,object_name_[oID]));
//      callLater(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
//      object_state_[oID] = true;
//    }
//  }

//  for(int i=0; i < MAX_ICS_OBJECT_NUM; i++){
//    if (object_state_[i]==false) callLater(bind(&TmsRpController::disappear,tac_,object_name_[i]));
//  }

  //----------------------------------------------------------------------------
  // update information of objects in refrigerator

//  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) object_state_[i]=false;

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) {
    getObjectData.request.tmsdb.id =7001 + i;
    getObjectData.request.tmsdb.sensor = 3018; //refrigerator, irs

    if (get_data_client_.call(getObjectData)) {
      os_ << "[TmsAction] Get info of object ID: " << getObjectData.request.tmsdb.id <<"  OK" << endl;
    } else {
      os_ << "[TmsAction] Failed to call service getObjectData ID: " << getObjectData.request.tmsdb.id << endl;
      return;
    }

    if (getObjectData.response.tmsdb.empty()==true) {
      os_ << "[TmsAction] nothing in refrigerator" <<endl;
      return;
    }

    if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==2009) {
      int    oID   = getObjectData.response.tmsdb[0].id - 7001;
      double oPosX = 4.5 - getObjectData.response.tmsdb[0].y/1000;
      double oPosY = 2.3 + getObjectData.response.tmsdb[0].x/1000;
      double oPosZ = 0.0 + getObjectData.response.tmsdb[0].z/1000;
      callSynchronously(bind(&TmsRpController::appear,tac_,object_name_[oID]));
      callSynchronously(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
      object_state_[oID] = true;
    }
    //----------------------------------------------------------------------------
    getObjectData.request.tmsdb.id =7001 + i;
    getObjectData.request.tmsdb.sensor = 3002; // ics

    if (get_data_client_.call(getObjectData)) {
      os_ << "[TmsAction] Get info of object ID: " << getObjectData.request.tmsdb.id <<"  OK" << endl;
    } else {
      os_ << "[TmsAction] Failed to call service getObjectData ID: " << getObjectData.request.tmsdb.id << endl;
      return;
    }

    if (getObjectData.response.tmsdb.empty()==true) {
      os_ << "[TmsAction] nothing in shelf" <<endl;
      return;
    }

    if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6010) {
      int    oID   = getObjectData.response.tmsdb[0].id - 7001;
      double oPosX = 4.3  - getObjectData.response.tmsdb[0].y/1000;
      double oPosY = 1.7 + getObjectData.response.tmsdb[0].x/1000;
      double oPosZ = 0.08 + getObjectData.response.tmsdb[0].z/1000;
      callSynchronously(bind(&TmsRpController::appear,tac_,object_name_[oID]));
      callSynchronously(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
      object_state_[oID] = true;
    }

    if(object_state_[i] == false) {
        getObjectData.request.tmsdb.id =7001 + i;
        getObjectData.request.tmsdb.sensor = 3005; // fake

        if (get_data_client_.call(getObjectData)) {
            os_ << "[TmsAction] Get info of object ID: " << getObjectData.request.tmsdb.id <<"  OK" << endl;
        } else {
            os_ << "[TmsAction] Failed to call service getObjectData ID: " << getObjectData.request.tmsdb.id << endl;
            return;
        }

        if (getObjectData.response.tmsdb.empty()==true) {
            os_ << "[TmsAction] nothing in big shelf" <<endl;
            return;
        }

        if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6011) {
            int    oID   = getObjectData.response.tmsdb[0].id - 7001;
            double oPosX = getObjectData.response.tmsdb[0].x/1000;
            double oPosY = getObjectData.response.tmsdb[0].y/1000;
            double oPosZ = getObjectData.response.tmsdb[0].z/1000;
            callSynchronously(bind(&TmsRpController::appear,tac_,object_name_[oID]));
            callSynchronously(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
            object_state_[oID] = true;
        }
    }
  }

  for(int i=0; i < MAX_ICS_OBJECT_NUM; i++){
    if (object_state_[i]==false) callLater(bind(&TmsRpController::disappear,tac_,object_name_[i]));
  }

  //----------------------------------------------------------------------------
  // update information of person in floor
#if PERSON==1
  tms_msg_db::TmsdbGetData getPersonData;

  getPersonData.request.tmsdb.id     = 1001;
  getPersonData.request.tmsdb.sensor = 3001;

  if (get_data_client_.call(getPersonData)) {
    os_ << "[TmsAction] Get info of object ID: " << getPersonData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getPersonData ID: " << getPersonData.request.tmsdb.id << endl;
    return;
  }

  if (getPersonData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] nothing on floor (ID="<< getPersonData.request.tmsdb.id <<")" <<endl;
    return;
  }

  if (getPersonData.response.tmsdb[0].state==1) {
    double pPosX = getPersonData.response.tmsdb[0].x/1000;
    double pPosY = getPersonData.response.tmsdb[0].y/1000;
    double pPosZ = 0.9;
    //Vector3 rpy (getPersonData.response.tmsdb[0].rr,getPersonData.response.tmsdb[0].rp,getPersonData.response.tmsdb[0].ry);
    Vector3 rpy(0,0,deg2rad(getPersonData.response.tmsdb[0].ry));
    Matrix3 rot = rotFromRpy(rpy);

    if(pPosX == 0.0 && pPosY == 0.0) {
      callLater(bind(&TmsRpController::disappear,tac_,"person_1"));
    }
    else{
      callLater(bind(&TmsRpController::appear,tac_,"person_1"));
      callLater(bind(&TmsRpController::setPos,tac_,"person_1",Vector3(pPosX,pPosY,pPosZ),rot));
    }
  }
#else
  callLater(bind(&TmsRpController::disappear,tac_,"person_1"));
#endif
  os_ << "[TmsAction] End of update!" << endl;
}

//------------------------------------------------------------------------------
void TmsRpBar::simulationInfoButtonClicked()
{
  PlanBase *pb = PlanBase::instance();

  if (!pb->robTag2Arm.size()) {
    os_ <<  "[TmsAction] set robot before updateinfo!" << endl;
    return;
  }

  os_ << "[TmsAction] Update TMS DB Information" << endl;

  //----------------------------------------------------------------------------
  // update information of robot in floor
  tms_msg_db::TmsdbGetData getRobotData;

  for(int32_t i=2001; i<=2011; i++) {
    if (i==2004 || i==2007 || i==2008 || i==2009 || i==2010) continue;
      getRobotData.request.tmsdb.id =i;// + i;
      getRobotData.request.tmsdb.sensor = 3005;

    if (get_data_client_.call(getRobotData)) {
      os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
    } else {
      os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
      continue;
    }

    int ref_i = 0;
    if (getRobotData.response.tmsdb.empty()==true) {
      os_ << "[TmsAction] nothing on floor (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
      continue;
    } else {
        for (int j=0; j<getRobotData.response.tmsdb.size()-1; j++) {
          if (getRobotData.response.tmsdb[j].time < getRobotData.response.tmsdb[j+1].time) ref_i = j+1;
        }
      }

    if (getRobotData.response.tmsdb[ref_i].state==1 || getRobotData.response.tmsdb[ref_i].state==2) {
      double rPosX = getRobotData.response.tmsdb[ref_i].x/1000;
      double rPosY = getRobotData.response.tmsdb[ref_i].y/1000;
      double rPosZ = getRobotData.response.tmsdb[ref_i].offset_z/1000;
      Vector3 rpy (deg2rad(getRobotData.response.tmsdb[ref_i].rr),deg2rad(getRobotData.response.tmsdb[ref_i].rp),deg2rad(getRobotData.response.tmsdb[ref_i].ry));
      Matrix3 rot = rotFromRpy(rpy);
      // calc joint position
      std::vector<double> seq_of_joint;
      if (getRobotData.response.tmsdb[ref_i].joint.empty()==false) {
          std::vector<std::string> s_seq_of_joint;
          s_seq_of_joint.clear();
          boost::split(s_seq_of_joint, getRobotData.response.tmsdb[ref_i].joint, boost::is_any_of(";"));

          seq_of_joint.clear();

          // string to double
          std::stringstream ss;
          double d_tmp;
          for (int i=0; i<s_seq_of_joint.size(); i++) {
            ss.clear();
            ss.str("");

            ss << s_seq_of_joint.at(i);
            ss >> d_tmp;
            seq_of_joint.push_back(deg2rad(d_tmp));
            //ROS_INFO("j[%d] = [%f]  ", i, seq_of_joint.at(i));
          }
      }

      if(getRobotData.request.tmsdb.id == 2001) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callSynchronously(bind(&TmsRpController::disappear,tac_,"smartpal4"));
        }
        else{
          callSynchronously(bind(&TmsRpController::appear,tac_,"smartpal4"));
          callSynchronously(bind(&TmsRpController::setPos,tac_,"smartpal4",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if(getRobotData.request.tmsdb.id == 2002) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callSynchronously(bind(&TmsRpController::disappear,tac_,"smartpal5_1"));
        }
        else{
          if (grasping_ == 0) {
              callSynchronously(bind(&TmsRpController::appear,tac_,"smartpal5_1"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          } else if (grasping_ == 2) {
              callSynchronously(bind(&TmsRpController::appear,tac_,"smartpal5_1"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }else {
              callSynchronously(bind(&TmsRpController::disappear,tac_,"smartpal5_1"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          // joint appear
          //int state;
          //callSynchronously(bind(&TmsRpController::setRobotPosture, tac_, 0, "2002", seq_of_joint, &state));
          }
      }
      else if(getRobotData.request.tmsdb.id == 2003) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callSynchronously(bind(&TmsRpController::disappear,tac_,"smartpal5_2"));
        }
        else{
          if (grasping_ == 0) {
              callSynchronously(bind(&TmsRpController::disappear,tac_,"smartpal5_2"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          } else if (grasping_ == 2) {
              callSynchronously(bind(&TmsRpController::appear,tac_,"smartpal5_2"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          } else {
              callSynchronously(bind(&TmsRpController::appear,tac_,"smartpal5_2"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if(getRobotData.request.tmsdb.id == 2005) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callSynchronously(bind(&TmsRpController::disappear,tac_,"kobuki"));
        }
        else{
          callSynchronously(bind(&TmsRpController::appear,tac_,"kobuki"));
          callSynchronously(bind(&TmsRpController::setPos,tac_,"kobuki",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if(getRobotData.request.tmsdb.id == 2006) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callSynchronously(bind(&TmsRpController::disappear,tac_,"kxp"));
        }
        else{
          if (grasping_ == 0) {
              callSynchronously(bind(&TmsRpController::appear,tac_,"kxp"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          } else if (grasping_ == 2) {
              callSynchronously(bind(&TmsRpController::appear,tac_,"kxp"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }else {
              callSynchronously(bind(&TmsRpController::disappear,tac_,"kxp"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if(getRobotData.request.tmsdb.id == 2011) {
        if(rPosX == 0.0 && rPosY == 0.0) {
          callSynchronously(bind(&TmsRpController::disappear,tac_,"kxp2"));
        }
        else{
          if (grasping_ == 0) {
              callSynchronously(bind(&TmsRpController::disappear,tac_,"kxp2"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          } else if (grasping_ == 2) {
              callSynchronously(bind(&TmsRpController::appear,tac_,"kxp2"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          } else {
              callSynchronously(bind(&TmsRpController::appear,tac_,"kxp2"));
              callSynchronously(bind(&TmsRpController::set_all_Pos,tac_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
    }
  }

  // update information of refrigerator
  getRobotData.request.tmsdb.id = 2009;
  getRobotData.request.tmsdb.sensor = 2009;

  if (get_data_client_.call(getRobotData)) {
    os_ << "[TmsAction] Get info of object ID: " << getRobotData.request.tmsdb.id <<"  OK" << endl;
  } else {
    os_ << "[TmsAction] Failed to call service getRobotData ID: " << getRobotData.request.tmsdb.id << endl;
    return;
  }

  if (getRobotData.response.tmsdb.empty()==true) {
    os_ << "[TmsAction] Error (ID="<< getRobotData.request.tmsdb.id <<")" <<endl;
    return;
  }

  if (getRobotData.response.tmsdb[0].state==1) {
    callSynchronously(bind(&TmsRpController::appear,tac_,"refrigerator"));
    callSynchronously(bind(&TmsRpController::disappear,tac_,"refrigerator_open"));
  } else if (getRobotData.response.tmsdb[0].state==2) {
    callSynchronously(bind(&TmsRpController::disappear,tac_,"refrigerator"));
    callSynchronously(bind(&TmsRpController::appear,tac_,"refrigerator_open"));
  }

  //----------------------------------------------------------------------------
  // update information of objects
  tms_msg_db::TmsdbGetData getObjectData;

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) object_state_[i]=false;

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) {
    getObjectData.request.tmsdb.id =7001 + i;

      if (get_data_client_.call(getObjectData)) {
        os_ << "[TmsAction] Get info of object ID: " << getObjectData.request.tmsdb.id <<"  OK" << endl;
      } else {
        os_ << "[TmsAction] Failed to call service getObjectData ID: " << getObjectData.request.tmsdb.id << endl;
        return;
      }

      int ref_i = 0;
      if (getObjectData.response.tmsdb.empty()==true) {
        os_ << "[TmsAction] nothing in shelf" <<endl;
        return;
      } else {
        for (int j=0; j<getObjectData.response.tmsdb.size()-1; j++) {
          if (getObjectData.response.tmsdb[j].time < getObjectData.response.tmsdb[j+1].time) ref_i = j+1;
        }
      }

      if (getObjectData.response.tmsdb[ref_i].state==1 && getObjectData.response.tmsdb[ref_i].place==6010) {
        int    oID   = getObjectData.response.tmsdb[ref_i].id - 7001;
        double oPosX = 3.66 + getObjectData.response.tmsdb[ref_i].y/1000;
        double oPosY = 1.87 - getObjectData.response.tmsdb[ref_i].x/1000;
        double oPosZ = 0.0 + getObjectData.response.tmsdb[ref_i].z/1000;
        callSynchronously(bind(&TmsRpController::appear,tac_,object_name_[oID]));
        callSynchronously(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
        object_state_[oID] = true;
      }
      else if(getObjectData.response.tmsdb[ref_i].state==1 && getObjectData.response.tmsdb[ref_i].place==5001) {
        int    oID   = getObjectData.response.tmsdb[ref_i].id - 7001;
        double oPosX = getObjectData.response.tmsdb[ref_i].x/1000;
        double oPosY = getObjectData.response.tmsdb[ref_i].y/1000;
        double oPosZ = getObjectData.response.tmsdb[ref_i].z/1000;
        callSynchronously(bind(&TmsRpController::appear,tac_,object_name_[oID]));
        callSynchronously(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
        object_state_[oID] = true;
      }
      // object's place = SmartPal5(2002)
      else if(getObjectData.response.tmsdb[ref_i].state==2 && (getObjectData.response.tmsdb[ref_i].place==2002 || getObjectData.response.tmsdb[ref_i].place==2006)) {
        int    oID   = getObjectData.response.tmsdb[ref_i].id - 7001;
          double oPosX = getObjectData.response.tmsdb[ref_i].x/1000;
          double oPosY = getObjectData.response.tmsdb[ref_i].y/1000;
          double oPosZ = getObjectData.response.tmsdb[ref_i].z/1000;
          Vector3 rpy (deg2rad(getObjectData.response.tmsdb[ref_i].rr),deg2rad(getObjectData.response.tmsdb[ref_i].rp),deg2rad(getObjectData.response.tmsdb[ref_i].ry));
          Matrix3 rot = rotFromRpy(rpy);
          callSynchronously(bind(&TmsRpController::appear,tac_,object_name_[oID]));
          callSynchronously(bind(&TmsRpController::setPos,tac_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), rot));
        object_state_[oID] = true;
      }
    }

    for(int i=0; i < MAX_ICS_OBJECT_NUM; i++){
      if (object_state_[i]==false) callSynchronously(bind(&TmsRpController::disappear,tac_,object_name_[i]));
    }

    // update information of person in floor
  #if PERSON==1
    tms_msg_db::TmsdbGetData getPersonData;

    getPersonData.request.tmsdb.id     = 1001;
    getPersonData.request.tmsdb.sensor = 3001;

    if (get_data_client_.call(getPersonData)) {
      os_ << "[TmsAction] Get info of object ID: " << getPersonData.request.tmsdb.id <<"  OK" << endl;
    } else {
      os_ << "[TmsAction] Failed to call service getPersonData ID: " << getPersonData.request.tmsdb.id << endl;
      return;
    }

    if (getPersonData.response.tmsdb.empty()==true) {
      os_ << "[TmsAction] nothing on floor (ID="<< getPersonData.request.tmsdb.id <<")" <<endl;
      return;
    }

    if (getPersonData.response.tmsdb[0].state==1) {
      double pPosX = getPersonData.response.tmsdb[0].x/1000;
      double pPosY = getPersonData.response.tmsdb[0].y/1000;
      double pPosZ = 0.9;
      //Vector3 rpy (getPersonData.response.tmsdb[0].rr,getPersonData.response.tmsdb[0].rp,getPersonData.response.tmsdb[0].ry);
      Vector3 rpy(0,0,deg2rad(getPersonData.response.tmsdb[0].ry));
      Matrix3 rot = rotFromRpy(rpy);

      if(pPosX == 0.0 && pPosY == 0.0) {
        callSynchronously(bind(&TmsRpController::disappear,tac_,"person_1"));
      }
      else{
        callSynchronously(bind(&TmsRpController::appear,tac_,"person_1"));
        callSynchronously(bind(&TmsRpController::setPos,tac_,"person_1",Vector3(pPosX,pPosY,pPosZ), rot));
      }
    }
  #else
    callSynchronously(bind(&TmsRpController::disappear,tac_,"person_1"));
  #endif
  os_ << "[TmsAction] End of update!" << endl;
}

//------------------------------------------------------------------------------
void TmsRpBar::initPoseButtonClicked()
{
  os_ << endl;
  os_ << "[TmsAction] Init pose of robot all joint" << endl;
  tms_msg_rc::rc_robot_control sp_control_srv;

  sp_control_srv.request.unit = UNIT_VEHICLE;
  sp_control_srv.request.cmd  = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(3);
  sp_control_srv.request.arg[0] = 1000;
  sp_control_srv.request.arg[1] = 1000;
  sp_control_srv.request.arg[2] = 0;

  if (sp5_control_client_.call(sp_control_srv)) {
    int8_t result = sp_control_srv.response.result;
    os_ << "[TmsAction] action vehicle result = " << (int)result << endl;
  } else {
    os_ << "Failed to call service sp5_control" << endl;
  }

  sp_control_srv.request.unit = UNIT_ARM_R;
  sp_control_srv.request.cmd  = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(8);
  sp_control_srv.request.arg[0] = 0;    // init pose = 0
  sp_control_srv.request.arg[1] = -10;  // init pose = -10
  sp_control_srv.request.arg[2] = 0;    // init pose = 0
  sp_control_srv.request.arg[3] = 0;    // init pose = 0
  sp_control_srv.request.arg[4] = 0;    // init pose = 0
  sp_control_srv.request.arg[5] = 0;    // init pose = 0
  sp_control_srv.request.arg[6] = 0;    // init pose = 0
  sp_control_srv.request.arg[7] = 10;   // init pose = 0

  if (sp5_control_client_.call(sp_control_srv)) {
    int8_t result = sp_control_srv.response.result;
    os_ << "[TmsAction] action arm_r result = " << (int)result << endl;
  } else {
    os_ << "Failed to call service sp5_control" << endl;
  }

  sp_control_srv.request.unit = UNIT_ARM_L;
  sp_control_srv.request.cmd  = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(8);
  sp_control_srv.request.arg[0] = 0;    // init pose = 0
  sp_control_srv.request.arg[1] = -10;  // init pose = -10
  sp_control_srv.request.arg[2] = 0;    // init pose = 0
  sp_control_srv.request.arg[3] = 0;    // init pose = 0
  sp_control_srv.request.arg[4] = 0;    // init pose = 0
  sp_control_srv.request.arg[5] = 0;    // init pose = 0
  sp_control_srv.request.arg[6] = 0;    // init pose = 0
  sp_control_srv.request.arg[7] = 10;   // init pose = 0

  if (sp5_control_client_.call(sp_control_srv)) {
    int8_t result = sp_control_srv.response.result;
    os_ << "[TmsAction] action arm_l result = " << (int)result << endl;
  } else {
    os_ << "Failed to call service sp5_control" << endl;
  }

  sp_control_srv.request.unit = UNIT_GRIPPER_R;
  sp_control_srv.request.cmd  = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(3);
  sp_control_srv.request.arg[0] = 0;  // init pose = 0
  sp_control_srv.request.arg[1] = 10;
  sp_control_srv.request.arg[2] = 10;

  if (sp5_control_client_.call(sp_control_srv)) {
    int8_t result = sp_control_srv.response.result;
    os_ << "[TmsAction] action gripper_r result = " << (int)result << endl;
  } else {
    os_ << "Failed to call service sp5_control" << endl;
  }

  sp_control_srv.request.unit = UNIT_GRIPPER_L;
  sp_control_srv.request.cmd  = CMD_MOVE_ABS;
  sp_control_srv.request.arg.resize(3);
  sp_control_srv.request.arg[0] = 0;  // init pose = 0
  sp_control_srv.request.arg[1] = 10;
  sp_control_srv.request.arg[2] = 10;

  if (sp5_control_client_.call(sp_control_srv)) {
    int8_t result = sp_control_srv.response.result;
    os_ << "[TmsAction] action gripper_l result = " << (int)result << endl;
  } else {
    os_ << "Failed to call service sp5_control" << endl;
  }

  sp_control_srv.request.unit = UNIT_LUMBA;
  sp_control_srv.request.cmd  = CMD_MOVE_REL;
  sp_control_srv.request.arg.resize(4);
  sp_control_srv.request.arg[0] = 0;  // init pose = 0
  sp_control_srv.request.arg[1] = 0;  // init pose = 0
  sp_control_srv.request.arg[2] = 10;
  sp_control_srv.request.arg[3] = 10;

  if (sp5_control_client_.call(sp_control_srv)) {
    int8_t result = sp_control_srv.response.result;
    os_ << "[TmsAction] action lumba result = " << (int)result << endl;
  } else {
    os_ << "Failed to call service sp5_control" << endl;
  }
}

//------------------------------------------------------------------------------
void TmsRpBar::moveToGoal()
{
  os_ << endl;
  os_ << "[TmsAction] Move to goal point" << endl;

  tms_msg_rp::rps_path_planning rplsrv;

  rplsrv.request.robot_id = 2002;
  rplsrv.request.rps_goal_candidate.rps_route.resize(1);
  rplsrv.request.rps_goal_candidate.rps_route[0].x  = goal_position_x_;
  rplsrv.request.rps_goal_candidate.rps_route[0].y  = goal_position_y_;
  rplsrv.request.rps_goal_candidate.rps_route[0].th = goal_position_ry_;

  if (path_planning_client_.call(rplsrv)) {
    os_ << "result: " << rplsrv.response.success << " message: " << rplsrv.response.message << endl;
  }
  else {
    os_ << "Failed to call service rps_path_planning" << endl;
    return;
  }

  if (rplsrv.response.rps_path.size()!=0) {
    for (int i=0; i< rplsrv.response.rps_path[0].rps_route.size(); i++) {
      os_ << "[" << i << "]: x="  << rplsrv.response.rps_path[0].rps_route[i].x <<
                          " y="  << rplsrv.response.rps_path[0].rps_route[i].y <<
                          " th=" << rplsrv.response.rps_path[0].rps_route[i].th << endl;
    }
  } else {
      os_ << "Noting rps_path_planning data" << endl;
      return;
  }

  tms_msg_rc::rc_robot_control sp_control_srv;

  if (rplsrv.response.rps_path.size()!=0) {
    for (int i=0; i< rplsrv.response.rps_path[0].rps_route.size(); i++) {
      sp_control_srv.request.unit = UNIT_VEHICLE;
      sp_control_srv.request.cmd  = CMD_MOVE_ABS;
      sp_control_srv.request.arg.resize(3);
      sp_control_srv.request.arg[0] = rplsrv.response.rps_path[0].rps_route[i].x;
      sp_control_srv.request.arg[1] = rplsrv.response.rps_path[0].rps_route[i].y;
      sp_control_srv.request.arg[2] = rplsrv.response.rps_path[0].rps_route[i].th;

      if (sp5_control_client_.call(sp_control_srv)) {
        int8_t result = sp_control_srv.response.result;
        os_ << "[TmsAction] action vehicle result = " << (int)result << endl;
      } else {
        os_ << "Failed to call service sp5_control" << endl;
      }
      simulationInfoButtonClicked();
      ros::spinOnce();
      sleep(1); //temp
    }
  }
}

//------------------------------------------------------------------------------
void TmsRpBar::startButtonClicked()
{

  clock_t start_ = clock();

  PlanBase *pb = PlanBase::instance();
  std::vector<pathInfo> trajectory;
  int state;
  std::vector<double> begin;
  std::vector<double> end;

  if( !pb->targetObject || !pb->robTag2Arm.size()) {
    os_ <<  "set object and robot" << endl;
    return;
  }

  for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as
    double q = pb->bodyItemRobot()->body()->joint(i)->q();
    begin.push_back(q);
    end.push_back(q);
  }
  TmsRpController::instance()->graspPathPlanStart(0, begin, end, pb->targetArmFinger->name, pb->targetObject->name(), 50, &trajectory, &state);
  os_ << pb->targetArmFinger->name << " " << pb->targetObject->name() << endl;
  clock_t end_ = clock();
  cout << "It took "<< (double)(end_-start_)/CLOCKS_PER_SEC << "(s)" << endl;

}

//------------------------------------------------------------------------------
void TmsRpBar::startButtonClicked2()
{
  PlanBase *pb = PlanBase::instance();
  std::vector<pathInfo> trajectory;
  int state;
  std::vector<double> begin;
  std::vector<double> end;

  if( !pb->targetObject || !pb->robTag2Arm.size()) {
    os_ <<  "set object and robot" << endl;
    return;
  }
  for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as
    double q = pb->bodyItemRobot()->body()->joint(i)->q();
    begin.push_back(q);
    end.push_back(q);
  }
  TmsRpController::instance()->releasePathPlanStart(0, begin, end,  pb->targetArmFinger->name, pb->targetObject->name(), 50, &trajectory, &state);
}

//------------------------------------------------------------------------------
void TmsRpBar::pathPlanButtonClicked()
{
  clock_t start_ = clock();

  PlanBase *pb = PlanBase::instance();
  std::vector<pathInfo> trajectory;
  int state;
  std::vector<double> begin;
  std::vector<double> end;

  if( !pb->robTag2Arm.size()) {
    os_ <<  "set object and robot _" << endl;
    return;
  }
  for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as
    double q = pb->bodyItemRobot()->body()->joint(i)->q();
    begin.push_back(q);
    end.push_back(q+1.0);
  }
  TmsRpController::instance()->pathPlanStart(0, begin, end, pb->targetArmFinger->name, "test", 50, &trajectory, &state);

  clock_t end_ = clock();
  cout << "It took "<< (double)(end_-start_)/CLOCKS_PER_SEC << "(s)" << endl;

  cout << "test" << endl;

}

//------------------------------------------------------------------------------
void TmsRpBar::changePlanningMode(){
  PlanBase::instance()->arm()->base_p = PlanBase::instance()->body()->link(0)->p();
  PlanBase::instance()->arm()->base_R = PlanBase::instance()->body()->link(0)->R();
  PlanBase::instance()->arm()->searchBasePositionMode = !PlanBase::instance()->arm()->searchBasePositionMode;
}

//------------------------------------------------------------------------------
void TmsRpBar::simulationButtonClicked(){
  os_ <<  "virtual button clicked" << endl;
  static boost::thread t(boost::bind(&TmsRpBar::simulation, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::connectRosButtonClicked(){
  os_ <<  "connectROS button clicked" << endl;
  static boost::thread t(boost::bind(&TmsRpBar::connectROS, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::simulation(){
  os_ <<  "simulation service" << endl;
  production_version_ = false;
  os_ << "production_version_ = " << production_version_ << endl;

  static ros::Rate loop_rate(10); // 0.1sec
  while (ros::ok())
  {
    if (planning_mode_ == 0)
      simulationInfoButtonClicked();

    ros::spinOnce();
    loop_rate.sleep();
  }
  printf("end of simulation");
}

//------------------------------------------------------------------------------
void TmsRpBar::connectROS(){
  os_ <<  "Connect to the ROS" << endl;
  production_version_ = true;
  os_ << "production_version_ = " << production_version_ << endl;
  tms_rp::TmsRpStaticMap StaticMap;

  static ros::Rate loop_rate(10); // 0.1sec
  while (ros::ok())
  {
    updateObjectInfo();

    StaticMap.MapPublish();

    ros::spinOnce();
    loop_rate.sleep();
  }
  printf("Disconnect to the ROS\n");
}

//------------------------------------------------------------------------------
void TmsRpBar::ardroneButtonClicked()
{
  Vector3 ardrone_pos = Vector3(0,0,0);
  Matrix3 ardrone_ori;
  Vector3 ardrone_rpy;

  TmsRpController trc;
  BodyItemPtr item = trc.objTag2Item()["ardrone_goal_position"];
  if(!item){
    os_ << "Error: the tagId is not recorded " << "ardrone_goal_position" << endl;
  }
  ardrone_pos = item->body()->link(0)->p();
  ardrone_ori = item->body()->link(0)->R();
  item->calcForwardKinematics();
  item->notifyKinematicStateChange();

  ardrone_rpy = rpyFromRot(ardrone_ori);
  os_ <<  "[TmsAction] ardrone's goal position: " << ardrone_pos(0) <<", "<< ardrone_pos(1) <<", "<< ardrone_pos(2) << endl;
  os_ <<  "[TmsAction] ardrone's goal orientation: " << ardrone_rpy(0) <<", "<< ardrone_rpy(1) <<", "<< ardrone_rpy(2) <<  endl;

  tms_msg_rc::robot_control ardrone_control_srv;

  ardrone_control_srv.request.x = ardrone_pos(0) * 1000;
  ardrone_control_srv.request.y = ardrone_pos(1) * 1000;
  ardrone_control_srv.request.z = ardrone_pos(2) * 1000;
  ardrone_control_srv.request.ry = rad2deg(ardrone_rpy(2));

  if (ardrone_client_.call(ardrone_control_srv)) {
    bool result = ardrone_control_srv.response.status;
    os_ << "[TmsAction] ardrone result = " << result << endl;
  } else {
    os_ << "Failed to call service ardrone_control" << endl;
  }
}

//------------------------------------------------------------------------------
