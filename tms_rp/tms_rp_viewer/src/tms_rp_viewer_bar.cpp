#include <tms_rp_viewer_bar.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

bool RpViewerBar::isRosInit = false;

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
      ros::init(argc, argv, "tms_rp_viewer");
      isRosInit=true;
      cout << "Success: connecting roscore.." << endl;
    }
  }
  catch(...)
  {
    cout << "Error: ros init" << endl;
  }

  // ros nodehandle, topic, service init
  static ros::NodeHandle nh;
  subscribe_environment_information_  = nh.subscribe("/tms_db_publisher/db_publisher", 1,  &RpViewerBar::updateEnvironmentInformation, this);

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
  uint32_t id,sensor,state;

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

    if (is_simulation)
    {
      if (sensor != 3005) // ID of fake sensor for simulation
        continue;
    }
    else
    {
      if (sensor != 3001) // ID of Vicon sensor
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


    if (state==1 || state==2)
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
      {ROS_INFO("Run1...");
        if (rPosX == 0.0 && rPosY == 0.0)
        {
          callSynchronously(bind(&TmsRpController::disappear,trc_,"smartpal5_2"));
        }
        else
        {
          if (!is_simulation)
          {ROS_INFO("Run2...");
          ROS_INFO("x=%f, y=%f",rPosX, rPosY);
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
        if (state==1)
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
      }
      else if (id ==6019)
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
    }
  }

//  //----------------------------------------------------------------------------
//  // update information of refrigerator
//  id = 2009;
//  getRobotData.request.tmsdb.sensor = 2009;

//  if (!get_data_client_.call(getRobotData))
//  {
//    ROS_INFO("Failed to call service get information of ID: %d",id);
//  }
//  else if (getRobotData.response.tmsdb.empty()==true)
//  {
//    ROS_INFO("No information of ID #%d in DB",id);

//  }
//  else if (environment_information_.tmsdb[i].state==0)
//  {
//    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator"));
//    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator_open"));
//  }
//  else if (environment_information_.tmsdb[i].state==1)
//  {
//    callSynchronously(bind(&TmsRpController::appear,trc_,"refrigerator"));
//    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator_open"));
//  }
//  else if (environment_information_.tmsdb[i].state==2)
//  {
//    callSynchronously(bind(&TmsRpController::disappear,trc_,"refrigerator"));
//    callSynchronously(bind(&TmsRpController::appear,trc_,"refrigerator_open"));
//  }

//  //----------------------------------------------------------------------------
//  // update information of wagon
//  id = 6019;

//  if (is_simulation)
//    getRobotData.request.tmsdb.sensor = 3005; // ID of fake sensor for simulation
//  else
//    getRobotData.request.tmsdb.sensor = 3001; // ID of Vicon sensor


//  if (!get_data_client_.call(getRobotData))
//  {
//    ROS_INFO("Failed to call service get information of ID: %d",id);
//  }
//  else if (getRobotData.response.tmsdb.empty()==true)
//  {
//    ROS_INFO("No information of ID #%d in DB",id);
//    callLater(bind(&TmsRpController::disappear,trc_,"wagon"));
//  }
//  else if (environment_information_.tmsdb[i].state==1)
//  {
//    rPosX = environment_information_.tmsdb[i].x/1000;
//    rPosY = environment_information_.tmsdb[i].y/1000;
//    rPosZ = 0.35;
//    rot = grasp::rotFromRpy(deg2rad(environment_information_.tmsdb[i].rr),
//                            deg2rad(environment_information_.tmsdb[i].rp),
//                            deg2rad(environment_information_.tmsdb[i].ry));

//    if(rPosX == 0.0 && rPosY == 0.0)
//    {
//      callSynchronously(bind(&TmsRpController::disappear,trc_,"wagon"));
//    }
//    else
//    {
//      callSynchronously(bind(&TmsRpController::appear,trc_,"wagon"));
//      callSynchronously(bind(&TmsRpController::setPos,trc_,"wagon",Vector3(rPosX,rPosY,rPosZ), rot));
//    }
//  }

//  //----------------------------------------------------------------------------
//  // update information of objects in shelf
//  tms_msg_db::TmsdbGetData getObjectData;
//  int    oID;;
//  double oPosX, oPosY, oPosZ;

//  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++) trb_.object_state_[i]=false;

//  for(int32_t i=0; i < MAX_ICS_OBJECT_NUM; i++)
//  {
//    getObjectData.request.tmsdb.id =7001 + i;
//    getObjectData.request.tmsdb.sensor = 3018; //refrigerator, irs

//    if (!get_data_client_.call(getObjectData))
//    {
//      ROS_INFO("Failed to call service get information of ID: %d",getObjectData.request.tmsdb.id);
//      continue;
//    }

//    if (getObjectData.response.tmsdb.empty()==true)
//    {
//      ROS_INFO("Nothing in refrigerator");
//      continue;
//    }
//    else
//    {
//      if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==2009)
//      {
//        oID   = getObjectData.response.tmsdb[0].id - 7001;
//        oPosX = 4.5 - getObjectData.response.tmsdb[0].y/1000;
//        oPosY = 2.3 + getObjectData.response.tmsdb[0].x/1000;
//        oPosZ = 0.0 + getObjectData.response.tmsdb[0].z/1000;
//        callSynchronously(bind(&TmsRpController::appear,trc_,trb_.object_name_[oID]));
//        callSynchronously(bind(&TmsRpController::setPos,trc_,trb_.object_name_[oID],Vector3(oPosX,oPosY,oPosZ), trb_.mat_cw90_));
//        trb_.object_state_[oID] = true;
//      }
//    }

//    //----------------------------------------------------------------------------
//    getObjectData.request.tmsdb.id =7001 + i;
//    getObjectData.request.tmsdb.sensor = 3002; // shelf, ics

//    if (!get_data_client_.call(getObjectData))
//    {
//      ROS_INFO("Failed to call service get information of ID: %d",getObjectData.request.tmsdb.id);
//      continue;
//    }

//    if (getObjectData.response.tmsdb.empty()==true)
//    {
//      ROS_INFO("Nothing in shelf");
//      continue;
//    }
//    else
//    {
//      if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6010)
//      {
//        oID   = getObjectData.response.tmsdb[0].id - 7001;
//        oPosX = 4.3  - getObjectData.response.tmsdb[0].y/1000;
//        oPosY = 1.7 + getObjectData.response.tmsdb[0].x/1000;
//        oPosZ = 0.08 + getObjectData.response.tmsdb[0].z/1000;
//        callSynchronously(bind(&TmsRpController::appear,trc_,trb_.object_name_[oID]));
//        callSynchronously(bind(&TmsRpController::setPos,trc_,trb_.object_name_[oID],Vector3(oPosX,oPosY,oPosZ), trb_.mat_cw90_));
//        trb_.object_state_[oID] = true;
//      }
//    }

//    if(trb_.object_state_[i] == false)
//    {
//      getObjectData.request.tmsdb.id =7001 + i;
//      getObjectData.request.tmsdb.sensor = 3005; // fake

//      if (!get_data_client_.call(getObjectData))
//      {
//        ROS_INFO("Failed to call service get information of ID: %d",getObjectData.request.tmsdb.id);
//        continue;
//      }

//      if (getObjectData.response.tmsdb.empty()==true)
//      {
//        ROS_INFO("Nothing in big shelf");
//        continue;
//      }
//      else
//      {
//        if (getObjectData.response.tmsdb[0].state==1 && getObjectData.response.tmsdb[0].place==6011)
//        {
//          oID   = getObjectData.response.tmsdb[0].id - 7001;
//          oPosX = getObjectData.response.tmsdb[0].x/1000;
//          oPosY = getObjectData.response.tmsdb[0].y/1000;
//          oPosZ = getObjectData.response.tmsdb[0].z/1000;
//          callSynchronously(bind(&TmsRpController::appear,trc_,trb_.object_name_[oID]));
//          callSynchronously(bind(&TmsRpController::setPos,trc_,trb_.object_name_[oID],Vector3(oPosX,oPosY,oPosZ), trb_.mat_cw90_));
//          trb_.object_state_[oID] = true;
//        }
//      }
//    }
//  }

//  for(int i=0; i < MAX_ICS_OBJECT_NUM; i++)
//  {
//    if (trb_.object_state_[i]==false)
//    {
//      callSynchronously(bind(&TmsRpController::disappear,trc_,trb_.object_name_[i]));
//    }
//  }

//  // update information of person
//  #if PERSON==1
//  tms_msg_db::TmsdbGetData getPersonData;

//  getPersonData.request.tmsdb.id     = 1001;
//  getPersonData.request.tmsdb.sensor = 3001;

//  if (!get_data_client_.call(getPersonData))
//  {
//    ROS_INFO("Failed to call service get information of ID: %d",getPersonData.request.tmsdb.id);
//  }
//  else if (getPersonData.response.tmsdb.empty()==true)
//  {
//    ROS_INFO("no # %d person on floor", getPersonData.request.tmsdb.id);
//  }
//  else if (environment_information_.tmsdb[i].state==1)
//  {
//    oPosX = environment_information_.tmsdb[i].x/1000;
//    oPosY = environment_information_.tmsdb[i].y/1000;
//    oPosZ = 0.9;
//    rot = grasp::rotFromRpy(0,0,deg2rad(environment_information_.tmsdb[i].ry));

//    if(oPosX == 0.0 && oPosY == 0.0)
//    {
//      callSynchronously(bind(&TmsRpController::disappear,trc_,"person_1"));
//    }
//    else
//    {
//      callSynchronously(bind(&TmsRpController::appear,trc_,"person_1"));
//      callSynchronously(bind(&TmsRpController::setPos,trc_,"person_1",Vector3(oPosX,oPosY,oPosZ),rot));
//    }
//  }
//  #else
//    callSynchronously(bind(&TmsRpController::disappear,trc_,"person_1"));
//  #endif
}

//------------------------------------------------------------------------------
void RpViewerBar::rosOn()
{
  static ros::Rate loop_rate(30); // 0.1sec
  bool isTest=false;

  while (ros::ok())
  {
//    updateEnvironmentInformation(false);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
