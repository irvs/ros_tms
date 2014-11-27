#include <tms_rp_viewer_bar.h>

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
//                            trb_(*TmsRpBar::instance()),
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
  subscribe_environment_information_  = nh.subscribe("/tms_db_publisher/db_publisher", 1,  &RpViewerBar::updateEnvironmentInformation, this);

  // arrange model
  mat0_       <<  1, 0, 0, 0, 1, 0, 0, 0, 1;  //   0
  mat_ccw90_  <<  0,-1, 0, 1, 0, 0, 0, 0, 1;  //  90
  mat_ccw180_ << -1, 0, 0, 0,-1, 0, 0, 0, 1;  // 180
  mat_cw90_   <<  0, 1, 0,-1, 0, 0, 0, 0, 1;  // -90

  //------------------------------------------------------------------------------
  addSeparator();

  addLabel(("[RpViewerPlugin]"));

  addButton(("Viewer"), ("Viewer On"))->
    sigClicked().connect(bind(&RpViewerBar::onViewerClicked, this));
}

//------------------------------------------------------------------------------
RpViewerBar::~RpViewerBar()
{
}

//------------------------------------------------------------------------------
void RpViewerBar::onViewerClicked()
{
  static boost::thread t(boost::bind(&RpViewerBar::rosOn, this));
}

//------------------------------------------------------------------------------
void RpViewerBar::updateEnvironmentInformation(const tms_msg_db::TmsdbStamped::ConstPtr& msg)
{
  bool is_simulation = false;
  environment_information_ = *msg;

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
void RpViewerBar::rosOn()
{
  static ros::Rate loop_rate(30); // 0.1sec
  bool isTest=false;

  while (ros::ok())
  {
//    updateEnvironmentInformation(false);
    if (isTest==false)
    {
      callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"tv"));
      isTest=true;
    }
    else
    {
      callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"tv"));
      isTest=false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
