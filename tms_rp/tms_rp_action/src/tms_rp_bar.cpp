#include <tms_rp_bar.h>
#include <tms_rp_controller.h>
#include <tms_rp_voronoi_map.h>
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
                      trc_(*TmsRpController::instance()),
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
  request_robot_path_    = nh.serviceClient<tms_msg_rp::rps_voronoi_path_planning>("/rps_voronoi_path_planning");
  subscribe_pcd_         = nh.subscribe("velodyne_points", 10, &TmsRpBar::receivePointCloudData, this);
  subscribe_static_map_  = nh.subscribe("rps_map_data", 10,    &TmsRpBar::receiveStaticMapData, this);
  subscribe_dynamic_map_ = nh.subscribe("rps_dynamic_map", 10, &TmsRpBar::receiveDynamicMapData, this);
  subscribe_path_map_    = nh.subscribe("rps_robot_path", 10,  &TmsRpBar::receivePathMapData, this);
  subscribe_lrf_raw_data1_  = nh.subscribe("/urg1/most_intense", 10,  &TmsRpBar::receiveLrfRawData1, this);
  subscribe_lrf_raw_data2_  = nh.subscribe("/urg2/most_intense", 10,  &TmsRpBar::receiveLrfRawData2, this);
  subscribe_person_tracker_ = nh.subscribe("/tracking_points", 10,  &TmsRpBar::receivePersonTrackerInfo, this);

  //----------------------------------------------------------------------------
  group_lrf_raw_data_ = new SgInvariantGroup();
//  static boost::thread thread_viewLrfRaswdata(boost::bind(&TmsRpBar::viewLrfRawData, this));
//  thread_viewLrfRaswdata.join();

  //----------------------------------------------------------------------------
  // create person model
  trc_.createRecord(1001,"person_1");

  // create robot model
  trc_.createRecord(2001,"smartpal4");
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
  trc_.createRecord(6001,"big_sofa");
  trc_.createRecord(6002,"mini_sofa");
  trc_.createRecord(6003,"small_table");
  trc_.createRecord(6004,"tv_table");
  trc_.createRecord(6005,"tv");
  trc_.createRecord(6006,"partition1");
  trc_.createRecord(6007,"partition2");
  trc_.createRecord(6008,"partition3");
  trc_.createRecord(6009,"bed");
  trc_.createRecord(6010,"shelf");
  trc_.createRecord(6011,"big_shelf");
  trc_.createRecord(6012,"desk");
  trc_.createRecord(6013,"chair_desk");
  trc_.createRecord(6014,"table");
  trc_.createRecord(6015,"chair_table1");
  trc_.createRecord(6016,"chair_table2");
  trc_.createRecord(6017,"shelfdoor");
  trc_.createRecord(6018,"shelf2");
  trc_.createRecord(6019,"wagon");
  trc_.createRecord(6020,"sidetable");

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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_y)/1000;
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
  posX = (get_db_data.response.tmsdb[0].x)/1000;
    posY = (get_db_data.response.tmsdb[0].y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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
    posX = (get_db_data.response.tmsdb[0].x+get_db_data.response.tmsdb[0].offset_x)/1000;
    posY = (get_db_data.response.tmsdb[0].y+get_db_data.response.tmsdb[0].offset_y)/1000;
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

      posX = (get_db_data.response.tmsdb[0].x)/1000;
      posY = (get_db_data.response.tmsdb[0].y)/1000;
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
  for(int i=0; i<25; i++) {
    trc_.createRecord(7001+i, object_name_[i]);
  }

  //----------------------------------------------------------------------------
  trc_.createRobotRecord(2002,"smartpal5_1");
  //trc_.createRobotRecord(2006,"kxp");

  //------------------------------------------------------------------------------
  addLabel(("[ROS-TMS]"));

  addButton(QIcon(":/action/icons/simulation.png"), ("simulation mode"))
    ->sigClicked().connect(bind(&TmsRpBar::simulationButtonClicked, this));

  addButton(QIcon(":/action/icons/ros.png"), ("connect to the ros"))->
    sigClicked().connect(bind(&TmsRpBar::connectRosButtonClicked, this));

  addButton(QIcon(":/action/icons/ros.png"), ("test"))->
    sigClicked().connect(bind(&TmsRpBar::viewToggleClicked, this));

  addSeparator();

  addButton(QIcon(":/action/icons/collision_target.png"), ("set collision target model"))->
    sigClicked().connect(bind(&TmsRpBar::setCollisionTargetButtonClicked, this));

  addButton(QIcon(":/action/icons/collision_map.png"), ("make collision map"))->
    sigClicked().connect(bind(&TmsRpBar::makeCollisionMapButtonClicked, this));

  addSeparator();

  static_map_toggle_ = addToggleButton(QIcon(":/action/icons/static_map.png"), ("staic map"));
  static_map_toggle_->setChecked(false);

  dynamic_map_toggle_= addToggleButton(QIcon(":/action/icons/dynamic_map.png"), ("dynamic map"));
  dynamic_map_toggle_->setChecked(false);

  local_map_toggle_ = addToggleButton(QIcon(":/action/icons/local_map.png"), ("local map"));
  local_map_toggle_->setChecked(false);

  path_map_toggle_ = addToggleButton(QIcon(":/action/icons/path_map.png"), ("option of path view"));
  path_map_toggle_->setChecked(false);

  robot_map_toggle_ = addToggleButton(QIcon(":/action/icons/robot_map.png"), ("option of robot marker"));
  robot_map_toggle_->setChecked(false);

  addSeparator();

  point2d_toggle_ = addToggleButton(QIcon(":/action/icons/lrf_raw_data.png"), ("option of lrf raw data"));
  point2d_toggle_->setChecked(false);

  person_toggle_ = addToggleButton(QIcon(":/action/icons/person.png"), ("option of person marker"));
  person_toggle_->setChecked(false);

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
void TmsRpBar::receiveDynamicMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg)
{
  dynamic_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::receivePathMapData(const tms_msg_rp::rps_route::ConstPtr& msg)
{
  path_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::receiveLrfRawData1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lrf_raw_data1_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::receiveLrfRawData2(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lrf_raw_data2_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::receivePersonTrackerInfo(const tms_msg_ss::tracking_points::ConstPtr& msg)
{
  person_position_ = *msg;
}

//------------------------------------------------------------------------------
void TmsRpBar::getPcdData(){

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
void TmsRpBar::viewStaticMap()
{
  if(!static_map_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"static_map"));
    return;
  }

  if(static_map_data_.rps_map_x.size()==0)
  {
    ROS_INFO("nothing the static map data");
    return;
  }

  ROS_INFO("on view option for static map");

  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node = (SgGroup*)trc_.objTag2Item()["static_map"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0)
  {
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&static_map_data_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"static_map"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"static_map"));
}

//------------------------------------------------------------------------------
void TmsRpBar::viewDynamicMap()
{
  if(!dynamic_map_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"dynamic_map"));
    return;
  }

  if(dynamic_map_data_.rps_map_x.size()==0)
  {
    ROS_INFO("nothing the dynamic map data");
    return;
  }

  ROS_INFO("on view option for dynamic map");

  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node = (SgGroup*)trc_.objTag2Item()["dynamic_map"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0)
  {
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&dynamic_map_data_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"dynamic_map"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"dynamic_map"));
}

//------------------------------------------------------------------------------
void TmsRpBar::viewPathOfRobot()
{
  if(!path_map_toggle_->isChecked()){
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"path_map"));
    return;
  }

  if(path_map_data_.rps_route.size()==0){
    ROS_INFO("nothing the path map data for viewPathOfRobot");
    return;
  }

  ROS_INFO("on view option for path map");
  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node  = (SgGroup*)trc_.objTag2Item()["path_map"]->body()->link(0)->shape();

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

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"path_map"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"path_map"));
}

//------------------------------------------------------------------------------
void TmsRpBar::viewMarkerOfRobot()
{
  if(!robot_map_toggle_->isChecked()){
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"robot_marker"));
    return;
  }
    if(path_map_data_.rps_route.size()==0){
    ROS_INFO("nothing the path map data for robot_marker");
    return;
  }

  ROS_INFO("On view option for robot marker");
  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node  = (SgGroup*)trc_.objTag2Item()["robot_marker"]->body()->link(0)->shape();

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

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"robot_marker"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"robot_marker"));
}

//------------------------------------------------------------------------------
void TmsRpBar::viewLrfRawData()
{
  if(!point2d_toggle_->isChecked())
  {
    SceneView::instance()->removeEntity(group_lrf_raw_data_);
    return;
  }

  if(lrf_raw_data1_.ranges.size()==0)
  {
    ROS_INFO("nothing the LRF raw data 1");
    return;
  }

  ROS_INFO("on view option for LRF raw data");

  unsigned int num_points = lrf_raw_data1_.ranges.size();

  SgVertexArrayPtr vertices = new SgVertexArray();
  SgColorArrayPtr  colors   = new SgColorArray();

  vertices->reserve(num_points);
  float angle, distance, lrf_x, lrf_y;
  float lrf_set_x;
  float lrf_set_y;
  float lrf_set_z;
  lrf_set_x = 2.5;
  lrf_set_y = 0.15;
  lrf_set_z = 0.875;

  SgVector3 vertex;
  SgVector3 color;

  for(int i=3; i<num_points; ++i)
  {
    angle = (i * 0.25 - 45)* M_PI/180.;
    if(angle>=0 && angle<=M_PI)
    {
      distance = lrf_raw_data1_.ranges[i];
      lrf_x  = distance * cos(angle) + lrf_set_x;
      lrf_y  = distance * sin(angle) + lrf_set_y;
      vertex = SgVector3(lrf_x,lrf_y,lrf_set_z);
      color  = SgVector3(1.0,0.0,0.0);
      vertices->push_back(vertex);
      colors->push_back(color);
    }
  }

  SgPointSetPtr lrf_point_set = new SgPointSet();
  lrf_point_set->setPointSize(5.0);
  lrf_point_set->setColors(colors);
  lrf_point_set->setVertices(vertices);
  ROS_INFO("point size %d",lrf_point_set->pointSize());

  if(lrf_point_set)
  {
    group_lrf_raw_data_->addChild(lrf_point_set);
    SceneView::instance()->addEntity(group_lrf_raw_data_);
    ROS_INFO("points have been added %d",group_lrf_raw_data_->numChildren());
  }

}

//------------------------------------------------------------------------------
void TmsRpBar::viewLrfRawDataNew()
{
  if(!point2d_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"lrf_raw_data"));
    return;
  }

  if(lrf_raw_data1_.ranges.size()==0)
  {
    ROS_INFO("nothing the LRF raw data 1");
    return;
  }

  if(lrf_raw_data2_.ranges.size()==0)
  {
    ROS_INFO("nothing the LRF raw data 2");
    return;
  }


  ROS_INFO("on view option for LRF raw data");

  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node = (SgGroup*)trc_.objTag2Item()["lrf_raw_data"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0)
  {
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&lrf_raw_data1_,&lrf_raw_data2_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"lrf_raw_data"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"lrf_raw_data"));
}

//------------------------------------------------------------------------------
void TmsRpBar::viewPersonPostion()
{
  if(!person_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"person_tracker"));
    return;
  }

  if(person_position_.tracking_grid.size()==0)
  {
    ROS_INFO("nothing the person");
    return;
  }


  ROS_INFO("on view option for person tracker");

  SgPointsDrawing::SgLastRenderer(0,true);
  SgGroupPtr node = (SgGroup*)trc_.objTag2Item()["person_tracker"]->body()->link(0)->shape();

  SgPointsGet visit;
  node->accept(visit);
  if(visit.shape.size()==0)
  {
    ROS_INFO("no shape node, %ld", visit.shape.size());
    return;
  }

  SgPointsDrawing* cr = SgPointsDrawing::SgLastRenderer(0,false);
  cr = new SgPointsDrawing(&person_position_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"person_tracker"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"person_tracker"));
}

//------------------------------------------------------------------------------
void TmsRpBar::updateEnvironmentInfomation(bool is_simulation)
{
  PlanBase *pb = PlanBase::instance();
  double rPosX,rPosY,rPosZ;
  Matrix3 rot;

  if (!pb->robTag2Arm.size())
  {
    ROS_INFO("Please set a robot item before start");
    return;
  }
  else
  {
    if (is_simulation)
      ROS_DEBUG("Update ROS-TMS database information (simulation mode)");
    else
      ROS_DEBUG("Update ROS-TMS database information (ros mode)");
  }

  // update information of robot in the environment
  tms_msg_db::TmsdbGetData getRobotData;

  for (int32_t i=2001; i<=2011; i++) // robot IDs
  {
    if (is_simulation)  // simulation mode
    {
      // turtlebot2, ardrone, refrigerator, none
      if (i==2004 || i==2008 || i==2009 || i==2010) continue;
    }
    else                // real world mode
    {
      if (i==2005)
        callSynchronously(bind(&TmsRpController::disappear,trc_,"kobuki"));
      if (i==2006)
        callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp"));
      if (i==2011)
        callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp2"));
      // turtlebot2, kobuki, kxp, refrigerator, none, kxp2
      if (i==2004 || i==2005 || i==2006 || i==2009 || i==2010 || i==2011) continue;
    }

    getRobotData.request.tmsdb.id = i;

    if (is_simulation)
      getRobotData.request.tmsdb.sensor = 3005; // ID of fake sensor for simulation
    else
      getRobotData.request.tmsdb.sensor = 3001; // ID of Vicon sensor

    if (!get_data_client_.call(getRobotData))
    {
      ROS_INFO("Failed to call service get information of ID: %d",getRobotData.request.tmsdb.id);
      continue;
    }

    int32_t ref_i = 0;
    if (getRobotData.response.tmsdb.empty()==true)
    {
      ROS_INFO("No information of ID #%d in DB",getRobotData.request.tmsdb.id);
      continue;
    }
    else
    {
      if (is_simulation)
      {
        for (int j=0; j<getRobotData.response.tmsdb.size()-1; j++)
        {
          if (getRobotData.response.tmsdb[j].time < getRobotData.response.tmsdb[j+1].time)
            ref_i = j+1;
        }
      }
      else
      {
        ref_i = 0;
      }
    }

    if (getRobotData.response.tmsdb[ref_i].state==1 || getRobotData.response.tmsdb[ref_i].state==2)
    {
      rPosX = getRobotData.response.tmsdb[ref_i].x/1000;
      rPosY = getRobotData.response.tmsdb[ref_i].y/1000;
      rPosZ = getRobotData.response.tmsdb[ref_i].offset_z/1000;
      rot = rotFromRpy(deg2rad(getRobotData.response.tmsdb[ref_i].rr),
                       deg2rad(getRobotData.response.tmsdb[ref_i].rp),
                       deg2rad(getRobotData.response.tmsdb[ref_i].ry));

      // calc joint position
      std::vector<double> seq_of_joint;
      if (getRobotData.response.tmsdb[ref_i].joint.empty()==false && is_simulation)
      {
        std::vector<std::string> s_seq_of_joint;
        s_seq_of_joint.clear();
        boost::split(s_seq_of_joint, getRobotData.response.tmsdb[ref_i].joint, boost::is_any_of(";"));
        seq_of_joint.clear();

        // string to double
        std::stringstream ss;
        double d_tmp;
        for (int i=0; i<s_seq_of_joint.size(); i++)
        {
          ss.clear();
          ss.str("");
          ss << s_seq_of_joint.at(i);
          ss >> d_tmp;
          seq_of_joint.push_back(deg2rad(d_tmp));
        }
      }

      // set position of robot and decide the appear
      if (getRobotData.request.tmsdb.id == 2001)
      {
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal4"));
        }
        else
        {
          callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal4"));
          callSynchronously(bind(&TmsRpController::setPos,trc_,"smartpal4",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if (getRobotData.request.tmsdb.id == 2002)
      {
        if(rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_1"));
        }
        else
        {
          if (!is_simulation)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_1"));
            callSynchronously(bind(&TmsRpController::setPos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot));
          }
          else if (grasping_ == 0)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_1"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else if (grasping_ == 2)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_1"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else
          {
            callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_1"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (getRobotData.request.tmsdb.id == 2003)
      {
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_2"));
        }
        else
        {
          if (!is_simulation)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_2"));
            callSynchronously(bind(&TmsRpController::setPos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot));
          }
          else if (grasping_ == 0)
          {
            callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else if (grasping_ == 2)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (getRobotData.request.tmsdb.id == 2005)
      {
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"kobuki"));
        }
        else
        {
          callSynchronously(bind(&TmsRpController::appear,trc_,"kobuki"));
          callSynchronously(bind(&TmsRpController::setPos,trc_,"kobuki",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if (getRobotData.request.tmsdb.id == 2006)
      {
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp"));
        }
        else
        {
          if (!is_simulation)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp"));
            callSynchronously(bind(&TmsRpController::setPos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot));
          }
          else if (grasping_ == 0)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else if (grasping_ == 2)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else
          {
            callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (getRobotData.request.tmsdb.id == 2007)
      {
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"wheelchair"));
        }
        else
        {
          callSynchronously(bind(&TmsRpController::appear,trc_,"wheelchair"));
          callSynchronously(bind(&TmsRpController::setPos,trc_,"wheelchair",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if (getRobotData.request.tmsdb.id == 2008)
      {
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"ardrone"));
        }
        else
        {
          callSynchronously(bind(&TmsRpController::appear,trc_,"ardrone"));
          callSynchronously(bind(&TmsRpController::setPos,trc_,"ardrone",Vector3(rPosX,rPosY,rPosZ), rot));
        }
      }
      else if (getRobotData.request.tmsdb.id == 2011)
      {
        if(rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp2"));
        }
        else
        {
          if (!is_simulation)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp2"));
            callSynchronously(bind(&TmsRpController::setPos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot));
          }
          else if (grasping_ == 0)
          {
            callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else if (grasping_ == 2)
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
          else
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
    }
  }

  //----------------------------------------------------------------------------
  // update information of refrigerator
  getRobotData.request.tmsdb.id = 2009;
  getRobotData.request.tmsdb.sensor = 2009;

  if (!get_data_client_.call(getRobotData))
  {
    ROS_INFO("Failed to call service get information of ID: %d",getRobotData.request.tmsdb.id);
  }
  else if (getRobotData.response.tmsdb.empty()==true)
  {
    ROS_INFO("No information of ID #%d in DB",getRobotData.request.tmsdb.id);

  }
  else if (getRobotData.response.tmsdb[0].state==0)
  {
    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator"));
    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator_open"));
  }
  else if (getRobotData.response.tmsdb[0].state==1)
  {
    callSynchronously(bind(&TmsRpController::appear,trc_,"refrigerator"));
    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator_open"));
  }
  else if (getRobotData.response.tmsdb[0].state==2)
  {
    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator"));
    callSynchronously(bind(&TmsRpController::appear,trc_,"refrigerator_open"));
  }

  //----------------------------------------------------------------------------
  // update information of wagon
  getRobotData.request.tmsdb.id = 6019;

  if (is_simulation)
    getRobotData.request.tmsdb.sensor = 3005; // ID of fake sensor for simulation
  else
    getRobotData.request.tmsdb.sensor = 3001; // ID of Vicon sensor


  if (!get_data_client_.call(getRobotData))
  {
    ROS_INFO("Failed to call service get information of ID: %d",getRobotData.request.tmsdb.id);
  }
  else if (getRobotData.response.tmsdb.empty()==true)
  {
    ROS_INFO("No information of ID #%d in DB",getRobotData.request.tmsdb.id);
    callLater(bind(&TmsRpController::disappear,trc_,"wagon"));
  }
  else if (getRobotData.response.tmsdb[0].state==1)
  {
    rPosX = getRobotData.response.tmsdb[0].x/1000;
    rPosY = getRobotData.response.tmsdb[0].y/1000;
    rPosZ = 0.35;
    rot = rotFromRpy(deg2rad(getRobotData.response.tmsdb[0].rr),
                     deg2rad(getRobotData.response.tmsdb[0].rp),
                     deg2rad(getRobotData.response.tmsdb[0].ry));

    if(rPosX == 0.0 && rPosY == 0.0)
    {
      callSynchronously(bind(&TmsRpController::disappear,trc_,"wagon"));
    }
    else
    {
      callSynchronously(bind(&TmsRpController::appear,trc_,"wagon"));
      callSynchronously(bind(&TmsRpController::setPos,trc_,"wagon",Vector3(rPosX,rPosY,rPosZ), rot));
    }
  }

  //----------------------------------------------------------------------------
  // update information of objects in shelf
  tms_msg_db::TmsdbGetData getObjectData;
  int    oID;;
  double oPosX, oPosY, oPosZ;

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) object_state_[i]=false;

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++)
  {
    getObjectData.request.tmsdb.id =7001 + i;
    getObjectData.request.tmsdb.sensor = 3018; //refrigerator, irs

    if (!get_data_client_.call(getObjectData))
    {
      ROS_INFO("Failed to call service get information of ID: %d",getObjectData.request.tmsdb.id);
      continue;
    }

    if (getObjectData.response.tmsdb.empty()==true)
    {
      ROS_INFO("Nothing in refrigerator");
      continue;
    }
    else
    {
      if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==2009)
      {
        oID   = getObjectData.response.tmsdb[0].id - 7001;
        oPosX = 4.5 - getObjectData.response.tmsdb[0].y/1000;
        oPosY = 2.3 + getObjectData.response.tmsdb[0].x/1000;
        oPosZ = 0.0 + getObjectData.response.tmsdb[0].z/1000;
        callSynchronously(bind(&TmsRpController::appear,trc_,object_name_[oID]));
        callSynchronously(bind(&TmsRpController::setPos,trc_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
        object_state_[oID] = true;
      }
    }

    //----------------------------------------------------------------------------
    getObjectData.request.tmsdb.id =7001 + i;
    getObjectData.request.tmsdb.sensor = 3002; // shelf, ics

    if (!get_data_client_.call(getObjectData))
    {
      ROS_INFO("Failed to call service get information of ID: %d",getObjectData.request.tmsdb.id);
      continue;
    }

    if (getObjectData.response.tmsdb.empty()==true)
    {
      ROS_INFO("Nothing in shelf");
      continue;
    }
    else
    {
      if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6010)
      {
        oID   = getObjectData.response.tmsdb[0].id - 7001;
        oPosX = 4.3  - getObjectData.response.tmsdb[0].y/1000;
        oPosY = 1.7 + getObjectData.response.tmsdb[0].x/1000;
        oPosZ = 0.08 + getObjectData.response.tmsdb[0].z/1000;
        callSynchronously(bind(&TmsRpController::appear,trc_,object_name_[oID]));
        callSynchronously(bind(&TmsRpController::setPos,trc_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
        object_state_[oID] = true;
      }
    }

    if(object_state_[i] == false)
    {
      getObjectData.request.tmsdb.id =7001 + i;
      getObjectData.request.tmsdb.sensor = 3005; // fake

      if (!get_data_client_.call(getObjectData))
      {
        ROS_INFO("Failed to call service get information of ID: %d",getObjectData.request.tmsdb.id);
        continue;
      }

      if (getObjectData.response.tmsdb.empty()==true)
      {
        ROS_INFO("Nothing in big shelf");
        continue;
      }
      else
      {
        if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6011)
        {
          oID   = getObjectData.response.tmsdb[0].id - 7001;
          oPosX = getObjectData.response.tmsdb[0].x/1000;
          oPosY = getObjectData.response.tmsdb[0].y/1000;
          oPosZ = getObjectData.response.tmsdb[0].z/1000;
          callSynchronously(bind(&TmsRpController::appear,trc_,object_name_[oID]));
          callSynchronously(bind(&TmsRpController::setPos,trc_,object_name_[oID],Vector3(oPosX,oPosY,oPosZ), mat_cw90_));
          object_state_[oID] = true;
        }
      }
    }
  }

  for(int i=0; i < MAX_ICS_OBJECT_NUM; i++)
  {
    if (object_state_[i]==false)
    {
      callSynchronously(bind(&TmsRpController::disappear,trc_,object_name_[i]));
    }
  }

  // update information of person
  #if PERSON==1
  tms_msg_db::TmsdbGetData getPersonData;

  getPersonData.request.tmsdb.id     = 1001;
  getPersonData.request.tmsdb.sensor = 3001;

  if (!get_data_client_.call(getPersonData))
  {
    ROS_INFO("Failed to call service get information of ID: %d",getPersonData.request.tmsdb.id);
  }
  else if (getPersonData.response.tmsdb.empty()==true)
  {
    ROS_INFO("no # %d person on floor", getPersonData.request.tmsdb.id);
  }
  else if (getPersonData.response.tmsdb[0].state==1)
  {
    oPosX = getPersonData.response.tmsdb[0].x/1000;
    oPosY = getPersonData.response.tmsdb[0].y/1000;
    oPosZ = 0.9;
    rot = rotFromRpy(0,0,deg2rad(getPersonData.response.tmsdb[0].ry));

    if(oPosX == 0.0 && oPosY == 0.0)
    {
      callSynchronously(bind(&TmsRpController::disappear,trc_,"person_1"));
    }
    else
    {
      callSynchronously(bind(&TmsRpController::appear,trc_,"person_1"));
      callSynchronously(bind(&TmsRpController::setPos,trc_,"person_1",Vector3(oPosX,oPosY,oPosZ),rot));
    }
  }
  #else
    callSynchronously(bind(&TmsRpController::disappear,trc_,"person_1"));
  #endif
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
      updateEnvironmentInfomation(true);
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
    os_ <<  "set object_ and robot" << endl;
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
    os_ <<  "set object_ and robot" << endl;
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
  static boost::thread t(boost::bind(&TmsRpBar::simulation, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::connectRosButtonClicked()
{
  os_ <<  "connectROS button clicked" << endl;
  static boost::thread t(boost::bind(&TmsRpBar::connectROS, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::viewToggleClicked()
{
  os_ <<  "viewToggle button clicked" << endl;
  static boost::thread t(boost::bind(&TmsRpBar::viewLrfRawData, this));
}

//------------------------------------------------------------------------------
void TmsRpBar::simulation()
{
  os_ <<  "simulation service" << endl;
  production_version_ = false;
  os_ << "production_version_ = " << production_version_ << endl;
  tms_rp::TmsRpVoronoiMap static_and_dynamic_map;

  static ros::Rate loop_rate(10); // 0.1sec
  while (ros::ok())
  {
    static_and_dynamic_map.staticMapPublish();
    static_and_dynamic_map.dynamicMapPublish();

    viewStaticMap();
    viewDynamicMap();
    viewPathOfRobot();
    viewMarkerOfRobot();

    if (planning_mode_ == 0)
      updateEnvironmentInfomation(true);

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
  os_ << "production_version_ = " << production_version_ << endl;
  tms_rp::TmsRpVoronoiMap static_and_dynamic_map;

  static ros::Rate loop_rate(100); // 0.01sec
  while (ros::ok())
  {
    ROS_INFO("rate test");
    updateEnvironmentInfomation(false);

    static_and_dynamic_map.staticMapPublish();
    static_and_dynamic_map.dynamicMapPublish(person_position_);

    viewStaticMap();
    viewDynamicMap();
    viewPathOfRobot();
    viewMarkerOfRobot();
    //viewLrfRawData();
    viewPersonPostion();

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

