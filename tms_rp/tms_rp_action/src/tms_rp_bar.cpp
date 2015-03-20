#include <tms_rp_bar.h>
#include <tms_rp_controller.h>
#include <tms_rp_voronoi_map.h>
#include <tms_rp_collision_map.h>
#include <sg_points_get.h>
#include <draw_points.h>
#include <tms_rp_pp.h>

#define BASIC_OFFSET_X 4700
#define BASIC_OFFSET_Y 500

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

std::string TmsRpBar::furniture_name_[22] = {"wardrobe","workdesk","drawer","chair","kitchen",
                                             "meeting_table","meeting_chair1","meeting_chair2","meeting_chair3","meeting_chair4",
                                             "meeting_chair5","partition","tv_table","tv_52inch","playrecoder",
                                             "sofa","sofa_table","bed","wagon","shelf",
                                             "tree","tv_multi"};

// initialize static variables
bool TmsRpBar::production_version_ = false;
bool TmsRpBar::is_ros_Init_ = false;

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
  x_llimit_.setRange(-20.00, 20.00);
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
  x_ulimit_.setRange(-20.00, 20.00);
  x_ulimit_.setSingleStep(0.01);
  x_ulimit_.setValue(15.00);
  hbox->addWidget(&x_ulimit_);
  hbox->addWidget(new QLabel("(m)"));
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel(" y_llimit_  "));
  y_llimit_.setAlignment(Qt::AlignCenter);
  y_llimit_.setDecimals(2);
  y_llimit_.setRange(-20.00, 20.00);
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
  y_ulimit_.setRange(-20.00, 20.00);
  y_ulimit_.setSingleStep(0.01);
  y_ulimit_.setValue(8.00);
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
  goal_pos_x_.setRange(0, 15000);
  goal_pos_x_.setValue(0);
  hbox->addWidget(&goal_pos_x_);
  hbox->addStretch();
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel("y:"));
  goal_pos_y_.setAlignment(Qt::AlignCenter);
  goal_pos_y_.setRange(0, 8000);
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
                      trc_(*TmsRpController::instance()),
                      argc_(), argv_() {
  // initialize ros connect
  try
  {
    if (!is_ros_Init_)
    {
      ros::init(argc_, argv_, "tms_rp_action", ros::init_options::AnonymousName);
      is_ros_Init_=true;
      ROS_INFO("[tms_rp_action] Success: connecting roscore.");
    }
  }
  catch(...)
  {
    ROS_INFO("[tms_rp_action] Error: ros init");
  }

  sid_ = 100000;

  // topic, service init
  get_data_client_       = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  sp5_control_client_    = nh.serviceClient<tms_msg_rc::rc_robot_control>("sp5_control");
  ardrone_client_        = nh.serviceClient<tms_msg_rc::robot_control>("robot_control");
  request_robot_path_    = nh.serviceClient<tms_msg_rp::rps_voronoi_path_planning>("/rps_voronoi_path_planning");
  subscribe_umo_tracker_ = nh.subscribe("/umo_tracking_points", 1,  &TmsRpBar::receiveUnknownMovingObjectTrackerInfo, this);

  nh.setParam("tms_rp_state", "none");
  nh.setParam("planning_mode", 0);
  nh.setParam("grasping", 0);
  //----------------------------------------------------------------------------
  group_lrf_raw_data_ = new SgInvariantGroup();

  //----------------------------------------------------------------------------
  // create person model
  trc_.createRecord(1001,"person_1");

  // create robot model
  //trc_.createRecord(2001,"smartpal4");
  trc_.createRecord(2002,"smartpal5_1");
  trc_.createRecord(2003,"smartpal5_2");
  //trc_.createRecord(2004,"turtlebot2");
  trc_.createRecord(2005,"kobuki");
  trc_.createRecord(2006,"kxp");
  trc_.createRecord(2007,"wheelchair");
  trc_.createRecord(2008,"ardrone");
  trc_.createRecord(2009,"refrigerator");
  trc_.createRecord(2010,"refrigerator_open");
  trc_.createRecord(2011,"kxp2");

  // create sensor model
  trc_.createRecord(3011,"portable_sensor_1");
  trc_.createRecord(3012,"portable_sensor_2");
  trc_.createRecord(3013,"portable_sensor_3");
  trc_.createRecord(3014,"portable_sensor_4");
  trc_.createRecord(3015,"portable_sensor_5");

  // create space model
  trc_.createRecord(5001,"floor928");
  trc_.createRecord(5002,"wall928");
  trc_.createRecord(5005,"corridor928");

  // create furniture model
  trc_.createRecord(6001,"wardrobe");
  trc_.createRecord(6002,"workdesk");
  trc_.createRecord(6003,"drawer");
  trc_.createRecord(6004,"chair");
  trc_.createRecord(6005,"kitchen");
  trc_.createRecord(6006,"meeting_table");
  trc_.createRecord(6007,"meeting_chair1");
  trc_.createRecord(6008,"meeting_chair2");
  trc_.createRecord(6009,"meeting_chair3");
  trc_.createRecord(6010,"meeting_chair4");
  trc_.createRecord(6011,"meeting_chair5");
  trc_.createRecord(6012,"partition");
  trc_.createRecord(6013,"tv_table");
  trc_.createRecord(6014,"tv_52inch");
  trc_.createRecord(6015,"playrecoder");
  trc_.createRecord(6016,"sofa");
  trc_.createRecord(6017,"sofa_table");
  trc_.createRecord(6018,"bed");
  trc_.createRecord(6019,"wagon");
  trc_.createRecord(6020,"shelf");
  trc_.createRecord(6021,"tree");
  trc_.createRecord(6022,"tv_multi");

  // create etc model
  trc_.createRecord(20001,"blink_arrow");
  trc_.createRecord(20002,"person_marker1");
  trc_.createRecord(20003,"person_marker2");
  trc_.createRecord(20004,"person_marker3");
  trc_.createRecord(20005,"person_marker4");
  trc_.createRecord(20006,"person_marker5");
  trc_.createRecord(20007,"ardrone_goal_position");
  trc_.createRecord(20008,"static_map");
  trc_.createRecord(20009,"dynamic_map");
  trc_.createRecord(20010,"local_map");
  trc_.createRecord(20011,"path_map");
  trc_.createRecord(20012,"robot_marker");
  trc_.createRecord(20013,"collision_target");
  trc_.createRecord(20014,"smartpal_goal");
  trc_.createRecord(20015,"lrf_raw_data");
  trc_.createRecord(20016,"person_tracker");

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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("person_1");
    trc_.setPos("person_1",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("person_1");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2001 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("smartpal4");
    trc_.setPos("smartpal4",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("smartpal4");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2002 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("smartpal5_1");
    trc_.setPos("smartpal5_1",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("smartpal5_1");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2003 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("smartpal5_2");
    trc_.setPos("smartpal5_2",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("smartpal5_2");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2005 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("kobuki");
    trc_.setPos("kobuki",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("kobuki");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2006 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("kxp");
    trc_.setPos("kxp",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    trc_.disappear("kxp2");
    trc_.setPos("kxp2",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("kxp");
    trc_.disappear("kxp2");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2007 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("wheelchair");
    trc_.setPos("wheelchair",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("wheelchair");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2008 + sid_;

  if (get_data_client_.call(get_db_data)) {
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("ardrone");
    trc_.setPos("ardrone",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    trc_.disappear("ardrone_goal_position");
    trc_.setPos("ardrone_goal_position",   Vector3(posX+1,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("ardrone");
    trc_.disappear("ardrone_goal_position");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 2009 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("refrigerator");
    trc_.setPos("refrigerator",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    trc_.disappear("refrigerator_open");
    trc_.setPos("refrigerator_open",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("refrigerator");
    trc_.disappear("refrigerator_open");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 5001 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z-get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("floor928");
    trc_.setPos("floor928",   Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("floor928");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 5002 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("wall928");
    trc_.setPos("wall928", Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("wall928");
    ROS_ERROR("[TmsAction] Failed to call service get_db_data");
  }

  //----------------------------------------------------------------------------
  get_db_data.request.tmsdb.id = 5005 + sid_;

  if (get_data_client_.call(get_db_data)){
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x+BASIC_OFFSET_X)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y+BASIC_OFFSET_Y)/1000;
    posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
    posRR=deg2rad(get_db_data.response.tmsdb[0].rr);
    posRP=deg2rad(get_db_data.response.tmsdb[0].rp);
    posRY=deg2rad(get_db_data.response.tmsdb[0].ry);

    trc_.appear("corridor928");
    trc_.setPos("corridor928", Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));
  } else {
    trc_.appear("corridor928");
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

      posX = (get_db_data.response.tmsdb[0].x+BASIC_OFFSET_X)/1000;
      posY = (get_db_data.response.tmsdb[0].y+BASIC_OFFSET_Y)/1000;
      posZ = (get_db_data.response.tmsdb[0].z+get_db_data.response.tmsdb[0].offset_z)/1000;
      posRR= deg2rad(get_db_data.response.tmsdb[0].rr);
      posRP= deg2rad(get_db_data.response.tmsdb[0].rp);
      posRY= deg2rad(get_db_data.response.tmsdb[0].ry);
      trc_.appear(furniture_name_[furnitureID]);
      trc_.setPos(furniture_name_[furnitureID],Vector3(posX,posY,posZ), Matrix3(rotFromRpy(Vector3(posRR, posRP,posRY))));

    } else {
      trc_.appear(furniture_name_[furnitureID]);
      os_ << "[TmsAction] Failed to call service get_db_data ID: " << get_db_data.request.tmsdb.id << endl;
    }
  }

  //----------------------------------------------------------------------------
  //  appear and setpos object_ model
  for(int i=0; i<MAX_ICS_OBJECT_NUM; i++) {
    trc_.createRecord(7001+i, object_name_[i]);
  }

  //----------------------------------------------------------------------------
  trc_.createRobotRecord(2002,"smartpal5_1");
  //trc_.createRobotRecord(2006,"kxp");

  //------------------------------------------------------------------------------
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

  addButton(QIcon(":/action/icons/drone.png"), ("drone"))->
    sigClicked().connect(bind(&TmsRpBar::ardroneButtonClicked, this));

  addButton(QIcon(":/action/icons/wheelchair.png"), ("wheelchair"))->
    sigClicked().connect(bind(&TmsRpBar::ardroneButtonClicked, this));

  addButton(QIcon(":/action/icons/smartpal.png"), ("smartpal"))->
    sigClicked().connect(bind(&TmsRpBar::smartpalButtonClicked, this));

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
void TmsRpBar::receiveUnknownMovingObjectTrackerInfo(const tms_msg_ss::tracking_points::ConstPtr& msg)
{
  unknown_moving_object_position_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::setCollisionTargetButtonClicked()
{
  if(target_body_items_.size()==1){
    cout << "target_body_items_ size = " << target_body_items_.size() << endl;
    TmsRpCollisionMap::instance()->setCollisionTarget(target_body_items_[0]);
    cout << TmsRpCollisionMap::instance()->collision_target_->name() << " is collision target"<< endl;
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
                                                  DirName,tr("CSV Files (*.csv)"));

  if (fileName.isEmpty())
  {
    ROS_INFO("File name is empty");
    return;
  }
  else
  {
    trc_.disappear("person_1");
    trc_.disappear("smartpal4");
    trc_.disappear("smartpal5_1");
    trc_.disappear("smartpal5_2");
    trc_.disappear("turtlebot2");
    trc_.disappear("kobuki");
    trc_.disappear("kxp");
    trc_.disappear("wheelchair");
    trc_.disappear("ardrone");
    trc_.disappear("kxp2");
    trc_.disappear("floor928");
    trc_.disappear("corridor928");
    trc_.disappear("wagon");

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
      ROS_INFO("Can not open the file");
      return;
    }

    SetMapParamDialog* SMPDialog = new SetMapParamDialog();
    SMPDialog->show();
    if(SMPDialog->exec()){}

    vector<vector<int> > collision_map;
    TmsRpCollisionMap::instance()->makeCollisionMap(collision_map);

    char temp[1024];
    sprintf(temp, "%f,%f,%f,%f,%f\n",
            TmsRpCollisionMap::instance()->x_llimit_, TmsRpCollisionMap::instance()->x_ulimit_,
            TmsRpCollisionMap::instance()->y_llimit_, TmsRpCollisionMap::instance()->y_ulimit_,
            TmsRpCollisionMap::instance()->cell_size_);
    file.write(temp);

    for(int x=0;x<collision_map.size();x++)
    {
      for(int y=0;y<collision_map[x].size();y++)
      {
        sprintf(temp, "%d,", collision_map[x][y] );
        file.write(temp);
      }
      file.write("\n");
    }
    file.close();
  }
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
void TmsRpBar::pathPlanButtonClicked()
{
  clock_t start_ = clock();

  PlanBase *pb = PlanBase::instance();
  std::vector<pathInfo> trajectory;
  int state;
  std::vector<double> begin;
  std::vector<double> end;

  if( !pb->robTag2Arm.size()) {
    os_ <<  "set object_ and robot _" << endl;
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
void TmsRpBar::changePlanningMode()
{
  PlanBase::instance()->arm()->base_p = PlanBase::instance()->body()->link(0)->p();
  PlanBase::instance()->arm()->base_R = PlanBase::instance()->body()->link(0)->R();
  PlanBase::instance()->arm()->searchBasePositionMode = !PlanBase::instance()->arm()->searchBasePositionMode;
}

//------------------------------------------------------------------------------
void TmsRpBar::simulationButtonClicked()
{
  os_ <<  "virtual button clicked" << endl;
  nh.setParam("tms_rp_state", "simulation");
  static boost::thread t(boost::bind(&TmsRpBar::simulation, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::connectRosButtonClicked()
{
  os_ <<  "connectROS button clicked" << endl;
  nh.setParam("tms_rp_state", "real");
  static boost::thread thread_connectROS(boost::bind(&TmsRpBar::connectROS, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::simulation()
{
  os_ <<  "simulation service" << endl;
  production_version_ = false;
  tms_rp::TmsRpVoronoiMap static_and_dynamic_map;

  static ros::Rate loop_rate(20); // 0.05sec
  while (ros::ok())
  {
    static_and_dynamic_map.staticMapPublish();
    static_and_dynamic_map.dynamicMapPublish();

    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("End of simulation");
}

//------------------------------------------------------------------------------
void TmsRpBar::connectROS()
{
  os_ <<  "Connect to the ROS" << endl;
  production_version_ = true;
  tms_rp::TmsRpVoronoiMap static_and_dynamic_map;

  static ros::Rate loop_rate(20); // 0.05sec
  while (ros::ok())
  {
    static_and_dynamic_map.staticMapPublish();
    static_and_dynamic_map.dynamicMapPublish(unknown_moving_object_position_);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Disconnect to the ROS\n");
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
void TmsRpBar::smartpalButtonClicked()
{
  boost::thread t(boost::bind(&TmsRpBar::moveSmartpal, this));
}


//------------------------------------------------------------------------------
void TmsRpBar::moveSmartpal()
{
  Vector3 robot_pos = Vector3(0,0,0);
  Matrix3 robot_ori;
  Vector3 robot_rpy;
  TmsRpController trc;
  BodyItemPtr item;
  tms_msg_rp::rps_voronoi_path_planning path_planning_srv;

  ROS_INFO("request the pah planning");

  // get current position of robot
  item = trc.objTag2Item()["smartpal5_2"];
  if(!item){
    ROS_INFO("Error: The tagId(smartpal5_2) is not recorded.");
  }
  robot_pos = item->body()->link(0)->p();
  robot_ori = item->body()->link(0)->R();
  item->calcForwardKinematics();
  item->notifyKinematicStateChange();

  robot_rpy = rpyFromRot(robot_ori);

  path_planning_srv.request.robot_id = 2003;
  path_planning_srv.request.start_pos.x = robot_pos(0) * 1000;
  path_planning_srv.request.start_pos.y = robot_pos(1) * 1000;
  path_planning_srv.request.start_pos.z = 0.0;
  path_planning_srv.request.start_pos.th = rad2deg(robot_rpy(2));
  path_planning_srv.request.start_pos.roll = 0.0;
  path_planning_srv.request.start_pos.pitch = 0.0;
  path_planning_srv.request.start_pos.yaw = rad2deg(robot_rpy(2));

  // get goal position of robot
  item = trc.objTag2Item()["smartpal_goal"];
  if(!item){
    ROS_INFO("Error: The tagId(smartpal_goal) is not recorded.");
  }
  robot_pos = item->body()->link(0)->p();
  robot_ori = item->body()->link(0)->R();
  item->calcForwardKinematics();
  item->notifyKinematicStateChange();

  robot_rpy = rpyFromRot(robot_ori);

  path_planning_srv.request.goal_pos.x = robot_pos(0) * 1000;
  path_planning_srv.request.goal_pos.y = robot_pos(1) * 1000;
  path_planning_srv.request.goal_pos.z = 0.0;
  path_planning_srv.request.goal_pos.th = rad2deg(robot_rpy(2));
  path_planning_srv.request.goal_pos.roll = 0.0;
  path_planning_srv.request.goal_pos.pitch = 0.0;
  path_planning_srv.request.goal_pos.yaw = rad2deg(robot_rpy(2));

  ROS_INFO("robot's current position: x=%f, y=%f",path_planning_srv.request.start_pos.x,
                                                  path_planning_srv.request.start_pos.y);
  ROS_INFO("robot's current theta: %f",path_planning_srv.request.start_pos.th);

  ROS_INFO("robot's goal position: x=%f, y=%f",path_planning_srv.request.goal_pos.x,
                                               path_planning_srv.request.goal_pos.y);
  ROS_INFO("robot's goal theta: %f",path_planning_srv.request.goal_pos.th);

  ROS_INFO("call request_robot_path");

  if (request_robot_path_.call(path_planning_srv))
  {
    uint32_t result = path_planning_srv.response.success;
    ROS_INFO("[TmsAction] path_planning_srv result = %d",result);
    return;
  }
  else
  {
    ROS_INFO("Failed to call service path_planning");
    return;
  }
}

//------------------------------------------------------------------------------

