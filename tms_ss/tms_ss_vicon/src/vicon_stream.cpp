//------------------------------------------------------------------------------
// @file   : vicon_stream.cpp
// @brief  : data stream using ViconSDK v1.3 / simple version
// @author : Yoonseok Pyo
// @version: Ver0.1.5 (since 2014.05.02)
// @date   : 2014.06.06
//------------------------------------------------------------------------------
#include <ros/ros.h>

#include <tms_msg_ss/vicon_data.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

#include <vicon_stream/client.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <time.h>

#define rad2deg(x) ((x)*(180.0)/M_PI)
#define deg2rad(x)  ((x)*M_PI/180.0)
//------------------------------------------------------------------------------
using std::string;
using namespace ViconDataStreamSDK::CPP;

//------------------------------------------------------------------------------
string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

//------------------------------------------------------------------------------
std::string Adapt( const bool i_Value )
{
  return i_Value ? "True" : "False";
}

//------------------------------------------------------------------------------
class ViconStream
{
//------------------------------------------------------------------------------  
private:
  // Sensor ID
  int32_t idSensor;
  // Place ID
  int32_t idPlace;
  // NodeHandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Publisher
  ros::Publisher db_pub;
  ros::Publisher pose_pub;
  ros::Publisher marker_pub;
  // Timer
  ros::Timer update_timer;
  // Parameters:
  string stream_mode;
  string host_name;
  string frame_id;
  double update_time;
  bool isDebug;
  // Access client for ViconSDK
  ViconDataStreamSDK::CPP::Client MyClient;

//------------------------------------------------------------------------------
public:
  ViconStream() : 
    nh_priv("~"),
    idSensor(3001),   // vicon sensor
    idPlace(5001),    // 988 room
    stream_mode("ClientPull"),
    host_name("192.168.4.175:801"),
    frame_id("/world"),
    update_time(0.01),  //sec
    isDebug(false)
  {
    // Init parameter
    nh_priv.param("stream_mode", stream_mode, stream_mode);
    nh_priv.param("host_name", host_name, host_name);
    nh_priv.param("frame_id", frame_id, frame_id);
    nh_priv.param("update_time_sec", update_time, update_time);
    nh_priv.param("debug", isDebug, isDebug);
    // Init Vicon Stream
    ROS_ASSERT(init_vicon());
    // Publishers
    db_pub    = nh.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data", 10);
    pose_pub  = nh_priv.advertise<tms_msg_ss::vicon_data> ("output", 10);
    // TimerEvent
    update_timer = nh.createTimer(ros::Duration(update_time), &ViconStream::updateCallback, this);
  }

  //----------------------------------------------------------------------------
  ~ViconStream()
  {
    ROS_ASSERT(shutdown_vicon());
  }

//------------------------------------------------------------------------------
private:
  bool init_vicon()
  {
    ROS_INFO_STREAM("Connecting to Vicon DataStream SDK at " << host_name << " ...");

    while (!MyClient.IsConnected().Connected)
    {
      MyClient.Connect(host_name);
      ROS_INFO(".");
      sleep(1);
      ros::spinOnce();
      if (!ros::ok())
        return false;
    }
    ROS_ASSERT(MyClient.IsConnected().Connected);
    ROS_INFO_STREAM("... connected!");
    ROS_INFO_STREAM("Setting Stream Mode to " << "ClientPull");

    Output_SetStreamMode result;
  
    if(stream_mode=="ClientPull")
    {
      result = MyClient.SetStreamMode(StreamMode::ClientPull);
    }
    else
    {
      ROS_INFO_STREAM("stream_mode error");
      return false;
    }

    if (result.Result != Result::Success)
    {
      ROS_FATAL("Set stream mode call failed -- shutting down");
      ros::shutdown();
    }

    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up); // 'Z-up'

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    ROS_INFO_STREAM(
                    "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
                        << Adapt(_Output_GetAxisMapping.YAxis) << " Z-" << Adapt(_Output_GetAxisMapping.ZAxis));
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    ROS_INFO_STREAM(
                    "Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
                        << _Output_GetVersion.Point);
    return true;
  }

  //----------------------------------------------------------------------------
  bool shutdown_vicon()
  {
    ROS_INFO_STREAM("Disconnecting from Vicon DataStream SDK");
    MyClient.Disconnect();
    ROS_ASSERT(!MyClient.IsConnected().Connected);
    ROS_INFO_STREAM("... disconnected.");
    return true;
  }

  //----------------------------------------------------------------------------
  void updateCallback(const ros::TimerEvent& e)
  {
    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();

    if(isDebug)
    {
      std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
      std::cout << "Marker Data Enabled: "           << Adapt( MyClient.IsMarkerDataEnabled().Enabled )          << std::endl;
      std::cout << "Unlabeled Marker Data Enabled: " << Adapt( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
      std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;
    }

    if(isDebug) std::cout << "Waiting for new frame...";
    while( MyClient.GetFrame().Result != Result::Success )
    {
      std::cout << "." << std::endl;
    }

    // Get the frame number
    Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
    if(isDebug) std::cout << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

    // Count the number of subjects
    unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
    if(isDebug) std::cout << "Subjects (" << SubjectCount << "):" << std::endl;

    ///////////////////////////////////////////////////////////////////
    for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
    {
      if(isDebug) std::cout << "  Subject #" << SubjectIndex << std::endl;

      // Get the subject name
      std::string SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
      if(isDebug) std::cout << "    Name: " << SubjectName << std::endl;

      // Get the root segment
      std::string RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
      if(isDebug) std::cout << "    Root Segment: " << RootSegment << std::endl;

      // Count the number of segments
      unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
      if(isDebug) std::cout << "    Segments (" << SegmentCount << "):" << std::endl;

      for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
      {
        if(isDebug) std::cout << "      Segment #" << SegmentIndex << std::endl;

        // Get the segment name
        std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
        if(isDebug) std::cout << "        Name: " << SegmentName << std::endl;

        // Get the global segment translation
        Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
          MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
        if(isDebug) std::cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[ 0 ]  << ", " 
                                                     << _Output_GetSegmentGlobalTranslation.Translation[ 1 ]  << ", " 
                                                     << _Output_GetSegmentGlobalTranslation.Translation[ 2 ]  << ") " 
                                                     << Adapt( _Output_GetSegmentGlobalTranslation.Occluded ) << std::endl;



        // Get the global segment rotation in quaternion co-ordinates
        Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = 
          MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
        if(isDebug) std::cout << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]     << ", " 
                                                             << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]     << ", " 
                                                             << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]     << ", " 
                                                             << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]     << ") " 
                                                             << Adapt( _Output_GetSegmentGlobalRotationQuaternion.Occluded ) << std::endl;

        // Get the global segment rotation in EulerXYZ co-ordinates
        Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = 
          MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
        if(isDebug) std::cout << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ]     << ", " 
                                                           << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ]     << ", " 
                                                           << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ]     << ") " 
                                                           << Adapt( _Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) << std::endl;


        tms_msg_ss::vicon_data pose_msg;
        
        ros::Time now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9

        pose_msg.header.frame_id  = frame_id;
        pose_msg.header.stamp     = now;
        pose_msg.measuredTime     = now;
        pose_msg.subjectName      = SubjectName;
        pose_msg.segmentName      = SegmentName;
        pose_msg.translation.x    = _Output_GetSegmentGlobalTranslation.Translation[0];
        pose_msg.translation.y    = _Output_GetSegmentGlobalTranslation.Translation[1];
        pose_msg.translation.z    = _Output_GetSegmentGlobalTranslation.Translation[2];
        pose_msg.rotation.x       = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
        pose_msg.rotation.y       = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
        pose_msg.rotation.z       = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
        pose_msg.rotation.w       = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
        pose_msg.eulerXYZ[0]      = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0];
        pose_msg.eulerXYZ[1]      = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1];
        pose_msg.eulerXYZ[2]      = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2];

        pose_pub.publish(pose_msg);

        std::cout << _Output_GetFrameNumber.FrameNumber << "::" << SegmentName <<std::endl;
        
        //----------------------------------------------------------------------
        // publish to tms_db_writer
        int32_t id = 0;

        if(SubjectName.compare("moverio") == 0)       id = 1001; //person_1
        else if(SubjectName.compare("sp4") == 0)      id = 2001;
        else if(SubjectName.compare("sp5_1") == 0)    id = 2002;
        else if(SubjectName.compare("sp5_2") == 0)    id = 2003;
        else if(SubjectName.compare("wheelchair") == 0)    id = 2007;   //added by Shigekane
        else if(SubjectName.compare("ardrone1") == 0) id = 2008;        
        else if(SubjectName.compare("oculus") == 0)   id = 3006;
        else if(SubjectName.compare("moverio") == 0)  id = 3016;
        else if(SubjectName.compare("wagon") == 0)    id = 6019;
        else id = -1;

        if(id != -1)
        {
          now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
          
          tms_msg_db::TmsdbStamped db_msg;
          tms_msg_db::Tmsdb tmpData;

          db_msg.header.frame_id  = frame_id;
          db_msg.header.stamp     = now;

          tmpData.time    = boost::posix_time::to_iso_extended_string(now.toBoost());
          tmpData.id      = id;
          tmpData.x       = pose_msg.translation.x;
          tmpData.y       = pose_msg.translation.y;
          tmpData.z       = pose_msg.translation.z;
          tmpData.rr      = rad2deg(pose_msg.eulerXYZ[0]);
          tmpData.rp      = rad2deg(pose_msg.eulerXYZ[1]);
          tmpData.ry      = rad2deg(pose_msg.eulerXYZ[2]);
          tmpData.place   = idPlace;
          tmpData.sensor  = idSensor;
          tmpData.state   = 1;

          db_msg.tmsdb.push_back(tmpData);
          
          db_pub.publish(db_msg);
        }
      }

      // unsigned int MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
      // std::cout << "    Markers (" << MarkerCount << "):" << std::endl;
      // for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
      // {
      //   // Get the marker name
      //   std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

      //   // Get the marker parent
      //   std::string MarkerParentName = MyClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

      //   // Get the global marker translation
      //   Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
      //     MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );

      //   std::cout << "      Marker #" << MarkerIndex            << ": "
      //                                 << MarkerName             << " ("
      //                                 << _Output_GetMarkerGlobalTranslation.Translation[ 0 ]  << ", "
      //                                 << _Output_GetMarkerGlobalTranslation.Translation[ 1 ]  << ", "
      //                                 << _Output_GetMarkerGlobalTranslation.Translation[ 2 ]  << ") "
      //                                 << std::endl;
      // }
    }
  }
};

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon_stream");
  ViconStream vs;
  ros::spin();
  return 0;
}

//------------------------------------------------------------------------------
//EOF
