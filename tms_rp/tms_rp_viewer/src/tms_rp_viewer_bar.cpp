#include <tms_rp_viewer_bar.h>
#include <sg_points_get.h>
#include <draw_points.h>


//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

//------------------------------------------------------------------------------
bool RpViewerBar::isRosInit = false;

std::string RpViewerBar::object_name_[25] = {"chipstar_red","chipstar_orange",
                                             "chipstar_green","greentea_bottle",
                                             "soukentea_bottle","cancoffee",
                                             "seasoner_bottle","dispenser",
                                             "soysauce_bottle_black","soysauce_bottle_blue",
                                             "soysauce_bottle_white","pepper_bottle_black",
                                             "pepper_bottle_red","sake_bottle",
                                             "teapot","chawan","teacup1","teacup2","cup1",
                                             "cup2","mugcup","remote","book_red","book_blue","dish"};

std::string RpViewerBar::furniture_name_[20] = {"big_sofa","mini_sofa","small_table","tv_table","tv",
                                                "partition1","partition2","partition3","bed","shelf",
                                                "big_shelf","desk","chair_desk","table","chair_table1","chair_table2",
                                                "shelfdoor","shelf2","wagon","sidetable"};


//------------------------------------------------------------------------------
RpViewerBar* RpViewerBar::instance(){
  static RpViewerBar* instance = new RpViewerBar();
  return instance;
}

//------------------------------------------------------------------------------
RpViewerBar::RpViewerBar(): ToolBar("RpViewerBar"),
                            mes(*MessageView::mainInstance()),
                            os(MessageView::mainInstance()->cout()),
                            trc_(*TmsRpController::instance()),
                            argc(), argv() 
{
  try
  {
    if (!isRosInit) 
    {
      ros::init(argc, argv, "tms_rp_viewer", ros::init_options::AnonymousName);
      isRosInit=true;
      ROS_INFO("[tms_rp_viewer] Success: connecting roscore.");
    }
  }
  catch(...)
  {
    ROS_INFO("[tms_rp_viewer] Error: ros init");
  }

  // ros nodehandle, topic, service init
  static ros::NodeHandle nh;

  subscribe_environment_information_  = nh.subscribe("/tms_db_publisher/db_publisher", 1,  &RpViewerBar::receiveEnvironmentInformation, this);

  subscribe_static_map_  = nh.subscribe("rps_map_data", 1,    &RpViewerBar::receiveStaticMapData, this);
  subscribe_dynamic_map_ = nh.subscribe("rps_dynamic_map", 1, &RpViewerBar::receiveDynamicMapData, this);
  subscribe_path_map_    = nh.subscribe("rps_robot_path", 1,  &RpViewerBar::receivePathMapData, this);
  subscribe_lrf_raw_data1_  = nh.subscribe("/urg1/most_intense", 1,  &RpViewerBar::receiveLrfRawData1, this);
  subscribe_lrf_raw_data2_  = nh.subscribe("/urg2/most_intense", 1,  &RpViewerBar::receiveLrfRawData2, this);
  subscribe_pcd_            = nh.subscribe("velodyne_points", 1, &RpViewerBar::receivePointCloudData, this);
  subscribe_umo_tracker_    = nh.subscribe("/umo_tracking_points", 1,  &RpViewerBar::receiveUnknownMovingObjectTrackerInfo, this);

  // arrange model
  mat0_       <<  1, 0, 0, 0, 1, 0, 0, 0, 1;  //   0
  mat_ccw90_  <<  0,-1, 0, 1, 0, 0, 0, 0, 1;  //  90
  mat_ccw180_ << -1, 0, 0, 0,-1, 0, 0, 0, 1;  // 180
  mat_cw90_   <<  0, 1, 0,-1, 0, 0, 0, 0, 1;  // -90

  //------------------------------------------------------------------------------
//  addButton(("Viewer"), ("Viewer On"))->
//    sigClicked().connect(bind(&RpViewerBar::onViewerClicked, this));

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

  addButton(("Viewer"), ("Viewer On"))->sigClicked().connect(bind(&RpViewerBar::onViewerClicked, this));
}

void RpViewerBar::onViewerClicked()
{
  static boost::thread t(boost::bind(&RpViewerBar::rosOn, this));
}

//------------------------------------------------------------------------------
RpViewerBar::~RpViewerBar()
{
}

//------------------------------------------------------------------------------
void RpViewerBar::receiveStaticMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg)
{
  static_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::receiveDynamicMapData(const tms_msg_rp::rps_map_full::ConstPtr& msg)
{
  dynamic_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::receivePathMapData(const tms_msg_rp::rps_route::ConstPtr& msg)
{
  path_map_data_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::receiveLrfRawData1(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lrf_raw_data1_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::receiveLrfRawData2(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  lrf_raw_data2_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::receivePointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*msg, cloud);
  point_cloud_data_ = cloud;
}

//------------------------------------------------------------------------------
void RpViewerBar::receiveUnknownMovingObjectTrackerInfo(const tms_msg_ss::tracking_points::ConstPtr& msg)
{
  unknown_moving_object_position_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::receiveEnvironmentInformation(const tms_msg_db::TmsdbStamped::ConstPtr& msg)
{
  environment_information_ = *msg;
}

//------------------------------------------------------------------------------
void RpViewerBar::updateEnvironmentInformation(bool is_simulation)
{
  PlanBase *pb = PlanBase::instance();
  double rPosX,rPosY,rPosZ;
  Matrix3 rot;
  uint32_t id,oID,sensor,state, place;
  bool object_state[MAX_ICS_OBJECT_NUM];

  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++)
    object_state[i] = false;

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

  if (environment_information_.tmsdb.size()==0)
    return;

  if (!is_simulation)
  {
    callSynchronously(bind(&TmsRpController::disappear,trc_,"kobuki"));
    callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp"));
    callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp2"));
  }

  for (uint32_t i=0; i<environment_information_.tmsdb.size(); i++)
  {
    id      = environment_information_.tmsdb[i].id;
    sensor  = environment_information_.tmsdb[i].sensor;
    state   = environment_information_.tmsdb[i].state;
    place   = environment_information_.tmsdb[i].place;

    if (is_simulation)  // simulation mode
    {
      // turtlebot2, ardrone, refrigerator, none
      if (id==2004 || id==2008 || id==2009 || id==2010) continue;
    }
    else                // real world mode
    {
      // turtlebot2, kobuki, kxp, refrigerator, none, kxp2
      if (id==2004 || id==2005 || id==2006 || id==2009 || id==2010 || id==2011) continue;
    }



    if (id == 2009 && sensor == 2009) // update information of refrigerator
    {
      if (state==0)
      {
        callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator"));
        callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator_open"));
      }
      else if (state==1)
      {
        callSynchronously(bind(&TmsRpController::appear,trc_,"refrigerator"));
        callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator_open"));
      }
      else if (state==2)
      {
        callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator"));
        callSynchronously(bind(&TmsRpController::appear,trc_,"refrigerator_open"));
      }
      continue;
    }

    if (is_simulation)
    {
      if (sensor == 3001) // ID of vicon
        continue;
    }
    else
    {
      if (sensor == 3003 || sensor == 3005) // ID of fake sensor for simulation
        continue;
    }

    int32_t ref_i = 0;

    if (is_simulation)
    {
//      for (int j=0; j<getRobotData.response.tmsdb.size()-1; j++)
//      {
//        if (getRobotData.response.tmsdb[j].time < getRobotData.response.tmsdb[j+1].time)
//          ref_i = j+1;
//      }
    }
    else
    {
      ref_i = 0;
    }


    if (state==1 || state==2) // ID of Vicon sensor
    {
      rPosX = environment_information_.tmsdb[i].x/1000;
      rPosY = environment_information_.tmsdb[i].y/1000;
      rPosZ = environment_information_.tmsdb[i].offset_z/1000;
      rot = grasp::rotFromRpy(deg2rad(environment_information_.tmsdb[i].rr),
                              deg2rad(environment_information_.tmsdb[i].rp),
                              deg2rad(environment_information_.tmsdb[i].ry));

      // calc joint position
      std::vector<double> seq_of_joint;
      if (environment_information_.tmsdb[i].joint.empty()==false && is_simulation)
      {
        std::vector<std::string> s_seq_of_joint;
        s_seq_of_joint.clear();
        boost::split(s_seq_of_joint, environment_information_.tmsdb[i].joint, boost::is_any_of(";"));
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
      if (id == 2001)
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
      else if (id == 2002)
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
//          else if (trb_.grasping_ == 0)
//          {
//            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_1"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
//          else if (trb_.grasping_ == 2)
//          {
//            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_1"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
          else
          {
            callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_1"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_1",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (id == 2003)
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
//          else if (trb_.grasping_ == 0)
//          {
//            callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_2"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
//          else if (trb_.grasping_ == 2)
//          {
//            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_2"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
          else
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"smartpal5_2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"smartpal5_2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (id == 2005)
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
      else if (id == 2006)
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
//          else if (trb_.grasping_ == 0)
//          {
//            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
//          else if (trb_.grasping_ == 2)
//          {
//            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
          else
          {
            callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (id == 2007)
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
      else if (id == 2008)
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
      else if (id == 2011)
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
//          else if (trb_.grasping_ == 0)
//          {
//            callSynchronously(bind(&TmsRpController::disappear,trc_,"kxp2"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
//          else if (trb_.grasping_ == 2)
//          {
//            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp2"));
//            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
//          }
          else
          {
            callSynchronously(bind(&TmsRpController::appear,trc_,"kxp2"));
            callSynchronously(bind(&TmsRpController::set_all_Pos,trc_,"kxp2",Vector3(rPosX,rPosY,rPosZ), rot, seq_of_joint));
          }
        }
      }
      else if (id ==1001)
      {
        rPosX = environment_information_.tmsdb[i].x/1000;
        rPosY = environment_information_.tmsdb[i].y/1000;
        rPosZ = 0.9;
        rot = grasp::rotFromRpy(0,0,deg2rad(environment_information_.tmsdb[i].ry));

        if(rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"person_1"));
        }
        else
        {
          callSynchronously(bind(&TmsRpController::appear,trc_,"person_1"));
          callSynchronously(bind(&TmsRpController::setPos,trc_,"person_1",Vector3(rPosX,rPosY,rPosZ),rot));
        }
      }
      else if (id ==6019) // wagon
      {
        if (state==1)
        {
          rPosX = environment_information_.tmsdb[i].x/1000;
          rPosY = environment_information_.tmsdb[i].y/1000;
          rPosZ = 0.35;
          rot = grasp::rotFromRpy(deg2rad(environment_information_.tmsdb[i].rr),
                                  deg2rad(environment_information_.tmsdb[i].rp),
                                  deg2rad(environment_information_.tmsdb[i].ry));

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
      }
      else if (id > 7000 && id < (7000 + MAX_ICS_OBJECT_NUM) && sensor == 3018 && place==2009) //refrigerator, irs
      {
        oID   = id - 7001;
        rPosX = 4.5 - environment_information_.tmsdb[i].y/1000;
        rPosY = 2.3 + environment_information_.tmsdb[i].x/1000;
        rPosZ = 0.0 + environment_information_.tmsdb[i].z/1000;
        callSynchronously(bind(&TmsRpController::appear,trc_,object_name_[oID]));
        callSynchronously(bind(&TmsRpController::setPos,trc_,object_name_[oID],Vector3(rPosX,rPosY,rPosZ), mat_cw90_));
        object_state[oID] = true;
      }
      else if (id > 7000 && id < (7000 + MAX_ICS_OBJECT_NUM) && sensor == 3002 && place==6010) //shelf, ics
      {
        oID   = id - 7001;
        rPosX = 4.3  - environment_information_.tmsdb[i].y/1000;
        rPosY = 1.7  + environment_information_.tmsdb[i].x/1000;
        rPosZ = 0.08 + environment_information_.tmsdb[i].z/1000;
        callSynchronously(bind(&TmsRpController::appear,trc_,object_name_[oID]));
        callSynchronously(bind(&TmsRpController::setPos,trc_,object_name_[oID],Vector3(rPosX,rPosY,rPosZ), mat_cw90_));
        object_state[oID] = true;
      }
    }
  }

  for (uint32_t i=0; i<environment_information_.tmsdb.size(); i++)
  {
    id      = environment_information_.tmsdb[i].id;
    if (id > 7000 && id < (7000 + MAX_ICS_OBJECT_NUM)) //fake
    {
      oID     = id - 7001;
      sensor  = environment_information_.tmsdb[i].sensor;
      state   = environment_information_.tmsdb[i].state;
      place   = environment_information_.tmsdb[i].place;

      if (object_state[oID] == false)
      {
        if (sensor == 3005 && place==6011 && state==1) //fake
        {
          rPosX = environment_information_.tmsdb[i].x/1000;
          rPosY = environment_information_.tmsdb[i].y/1000;
          rPosZ = environment_information_.tmsdb[i].z/1000;
          callSynchronously(bind(&TmsRpController::appear,trc_,object_name_[oID]));
          callSynchronously(bind(&TmsRpController::setPos,trc_,object_name_[oID],Vector3(rPosX,rPosY,rPosZ), mat_cw90_));
          object_state[oID] = true;
        }
      }
    }
  }

  for (uint32_t i=0; i < MAX_ICS_OBJECT_NUM; i++)
  {
    if (object_state[i] == false)
    {
      callSynchronously(bind(&TmsRpController::disappear,trc_,object_name_[i]));
    }
  }
}

//------------------------------------------------------------------------------
void RpViewerBar::viewStaticMap()
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
void RpViewerBar::viewDynamicMap()
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
void RpViewerBar::viewPathOfRobot()
{
  if(!path_map_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"path_map"));
    return;
  }

  if(path_map_data_.rps_route.size()==0)
  {
    ROS_INFO("nothing the path map data for viewPathOfRobot");
    return;
  }

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
void RpViewerBar::viewMarkerOfRobot()
{
  if(!robot_map_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"robot_marker"));
    return;
  }
    if(path_map_data_.rps_route.size()==0)
    {
    ROS_INFO("nothing the path map data for robot_marker");
    return;
  }

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
void RpViewerBar::viewLrfRawData()
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


  //ROS_INFO("on view option for LRF raw data");

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
void RpViewerBar::viewPersonPostion()
{
  if(!person_toggle_->isChecked())
  {
    callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"person_tracker"));
    return;
  }

  if(unknown_moving_object_position_.tracking_grid.size()==0)
  {
    ROS_INFO("nothing the person");
    return;
  }

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
  cr = new SgPointsDrawing(&unknown_moving_object_position_);
  visit.shape[0]->mesh()->triangles().clear();
  node->addChild(cr);

  callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"person_tracker"));
  callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"person_tracker"));
}
//------------------------------------------------------------------------------
void RpViewerBar::rosOn()
{
  static ros::Rate loop_rate(30); // 0.1sec
  bool isTest=false;

  while (ros::ok())
  {
    updateEnvironmentInformation(false);

    viewStaticMap();
    viewDynamicMap();
    viewPathOfRobot();
    viewMarkerOfRobot();
    viewLrfRawData();
    viewPersonPostion();

    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
