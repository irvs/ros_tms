//------------------------------------------------------------------------------
// @file   : object_selector.cpp
// @brief  : Return info of the object an user gazing
// @author : Kazuto Nakashima
// @date   : 2014.06.13
//------------------------------------------------------------------------------
// include for ROS
#include "ros/ros.h"
#include "tms_msg_db/TmsdbGetData.h"
#include "tms_ur_gaze_server/result.h"
#include "tms_ur_gaze_server/object_list.h"

// include for std
#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cstdlib>

// Eular angular definition
#define Roll  eular.x
#define Pitch eular.y
#define Yaw   eular.z
#define USER  1001    // M100:3016 Glasses:1001
#define SHOW  1       // Show the information of objects

// Mode parameter for calculating a key
int mode;

//------------------------------------------------------------------------------
// for something having three elements
class Triple {
public:
  double x;
  double y;
  double z;
};

//------------------------------------------------------------------------------
// Calculate the absolute value
double CalcAbs(Triple vector) {
  return sqrt(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
};

//------------------------------------------------------------------------------
// Calculate the inner product
double CalcInn(Triple vector, Triple vector2) {
  return vector.x*vector2.x + vector.y*vector2.y + vector.z*vector2.z;
}

//------------------------------------------------------------------------------
// Calculate the key for sorting objects
double CalcKey(double abs, double deg) {
  if(mode == 0) { return abs * deg; }
  else { return abs * (deg + 0.01); }
}

//------------------------------------------------------------------------------
// Glass class
class Glass {
public:
  // Posture info on the world coodinates
  Triple pos;
  Triple eular;
  // Vector on the glass coodinates
  Triple vector;
  double abs;
  Glass();
  void update();
};

Glass::Glass() {
  // Default (0, 0, 0)
  pos.x = 0;
  pos.y = 0;
  pos.z = 0;
  // Default (1, 0, 0)
  vector.x = 1;
  vector.y = 0;
  vector.z = 0;
  abs = CalcAbs(vector);
}

void Glass::update() {
  ros::NodeHandle nh;
  ros::ServiceClient get_data_client = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");
  
  tms_msg_db::TmsdbGetData getData;
  getData.request.tmsdb.id = USER;

  if (get_data_client.call(getData)) {
    ROS_INFO("Updated info of Glasses");
  } else {
    ROS_ERROR("Failed to call service (ID: %d)", getData.request.tmsdb.id);
    return;
  }

  if (getData.response.tmsdb.empty()==true) {
    ROS_ERROR("Nothing on floor ID: %d",getData.request.tmsdb.id);
    return;
  }

  // Store the odometry data from Vicon
  if (getData.response.tmsdb[0].state==1) {
    pos.x = getData.response.tmsdb[0].x/1000;
    pos.y = getData.response.tmsdb[0].y/1000;
    pos.z = getData.response.tmsdb[0].z/1000;
    Roll  = getData.response.tmsdb[0].rr * M_PI / 180;
    Pitch = getData.response.tmsdb[0].rp * M_PI / 180;
    Yaw   = getData.response.tmsdb[0].ry * M_PI / 180;
  } else {
    ROS_ERROR("Unavailable ID: %d", getData.request.tmsdb.id);
  }
  
  // Calculate the vector based on the eular angular
  vector.x = cos(Pitch) * cos(Yaw);
  vector.y = cos(Pitch) * sin(Yaw);
  vector.z = sin(Pitch);
  abs = CalcAbs(vector);
}

//------------------------------------------------------------------------------
// Object class
class Object {
private:
  //Triple pos;
public:
  std::string name; // Obejct name
  Triple pos;
  Triple offset_pos;
  Triple vector;    // Vector on the glass coodinates
  double abs;       // Absolute value of the vector
  double deg;       // The angle the object makes with glass
  double key;       // key for sorting
  int count;        // for dicision by majority
  int objectID;
  bool SetObject(Glass glass, int id);
  void update(Glass glass);
};
// Set elements of Object-class by calling the dbreader
bool Object::SetObject(Glass glass, int id) {
  ros::NodeHandle nh;
  ros::ServiceClient get_data_client = nh.serviceClient<tms_msg_db::TmsdbGetData>("/tms_db_reader/dbreader");

  tms_msg_db::TmsdbGetData getData;
  
  getData.request.tmsdb.id = id;

  if (get_data_client.call(getData)) {
    ROS_INFO("Get info of object ID: %d", getData.request.tmsdb.id);
  } else {
    ROS_ERROR("Failed to call service (ID: %d)", getData.request.tmsdb.id);
    return false;
  }

  if (getData.response.tmsdb.empty()==true) {
    ROS_ERROR("Nothing on floor ID: %d", getData.request.tmsdb.id);
    return false;
  }

  if (getData.response.tmsdb[0].state==1) {
    pos.x = getData.response.tmsdb[0].x/1000;
    pos.y = getData.response.tmsdb[0].y/1000;
    pos.z = getData.response.tmsdb[0].z/1000;
    offset_pos.x = getData.response.tmsdb[0].offset_x/1000;
    offset_pos.y = getData.response.tmsdb[0].offset_y/1000;
    offset_pos.z = getData.response.tmsdb[0].offset_z/1000;
    name  = getData.response.tmsdb[0].name;
  } else {
    ROS_ERROR("Unavailable ID: %d", getData.request.tmsdb.id);
    return false;
  }

  count = 0;
  vector.x = pos.x - glass.pos.x;
  vector.y = pos.y - glass.pos.y;
  vector.z = pos.z - glass.pos.z;
  objectID = id;
  abs = CalcAbs(vector);
  deg = CalcInn(vector, glass.vector) / (abs*CalcAbs(glass.vector));
  deg = acos(deg);
  key = CalcKey(abs,deg);

  return true;
}
// for static objects, never changes posture
void Object::update(Glass glass) {
  vector.x = pos.x - glass.pos.x;
  vector.y = pos.y - glass.pos.y;
  vector.z = pos.z - glass.pos.z;
  abs = CalcAbs(vector);
  deg = CalcInn(vector, glass.vector) / (abs*CalcAbs(glass.vector));
  deg = acos(deg);
  key = CalcKey(abs,deg);
}

//------------------------------------------------------------------------------
// Create a Glass-class and Object-class
Glass glass;
Object object_tmp;

//------------------------------------------------------------------------------
// Store addresses of object_data into the vector, 'object',
// and intend to manage or sort them in indirect referencing
void StoreAddress(std::vector<Object*>* object, std::vector<Object>* object_data) {
  std::vector<Object>::iterator it, begin, end;
  begin = object_data->begin();
  end   = object_data->end();
  for (it = begin; it != end; ++it) {
    object->push_back(&(*it));
  }
}

//------------------------------------------------------------------------------
// Update elements of class; Glass, Object
void Update(Glass *glass,
            std::vector<Object*>::iterator begin,
            std::vector<Object*>::iterator end) {
  std::vector<Object*>::iterator it;
  glass->update();
  for (it = begin; it != end; ++it) {
    (*it)->update(*glass);
  }
  ROS_INFO("Updated info of objects\n");
}

//------------------------------------------------------------------------------
// Show elements of class
// for Object-class
void Info(std::vector<Object*>::iterator begin,
          std::vector<Object*>::iterator end) {
  std::vector<Object*>::iterator it;
  for (it = begin; it != end; ++it) {
    std::cout << std::endl        << (*it)->name  << std::endl
              << "Value\t[count]" << (*it)->count
              << "\n\t[Abs]"      << (*it)->abs
              << "\n\t[Deg]"      << (*it)->deg * 180 / M_PI //degree
              << "\n\t[key]"      << CalcKey((*it)->abs,(*it)->deg)
              << std::endl;
    std::cout << "Pos\t[x]"       << (*it)->pos.x
              << "\n\t[y]"        << (*it)->pos.y
              << "\n\t[z]"        << (*it)->pos.z
              << std::endl;
    std::cout << "Vector\t[x]"    << (*it)->vector.x
              << "\n\t[y]"        << (*it)->vector.y
              << "\n\t[z]"        << (*it)->vector.z
              << std::endl;
  }
}
// for Glass-class
void Info(Glass *glass) {
  std::cout << "Glasses"
            << std::endl;
  std::cout << "Pos\t[x]"    << glass->pos.x
            << "\n\t[y]"     << glass->pos.y
            << "\n\t[z]"     << glass->pos.z
            << std::endl;
  std::cout << "Vector\t[x]" << glass->vector.x
            << "\n\t[y]"     << glass->vector.y
            << "\n\t[z]"     << glass->vector.z
            << std::endl;
}

//------------------------------------------------------------------------------
// Compare with the key value
bool Compare(const Object *object1, const Object *object2) {
  return object1->key < object2->key;
}
// Compare with the count number
bool Compare2(const Object *object1, const Object *object2) {
  return object1->count > object2->count;
}

//------------------------------------------------------------------------------
// Sort the possible objects
bool SortList(tms_ur_gaze_server::object_list::Request  &req, 
              tms_ur_gaze_server::object_list::Response &res) {
  ros::NodeHandle n;
  ros::Publisher display_pub = n.advertise<tms_ur_gaze_server::result>("tms_ur_gaze_server_answer", 10);

  tms_ur_gaze_server::result msg;

  std::vector<Object*> object;  // for indirect referencing
  std::vector<Object*>::iterator it, begin, end;
  std::vector<Object> object_data;  // Manage actual object data

  ros::Rate loop_rate(20);

  // Number for Dicisioin by majority
  int i = 1, j;

  ROS_INFO("Server has been called");

  // Create object_data table
  int size = req.id_in.size();
  for (j = 0; j < size; j++) {
    if (object_tmp.SetObject(glass, req.id_in[j]) != false) {
      object_data.push_back(object_tmp);
    }
  }

  // Create table stored addresses of object_data
  if (object_data.empty() == true) {
    ROS_ERROR("object_data contains nothing");
    return false;
  } else {
    StoreAddress(&object, &object_data);
  }

  // Set iterator
  begin = object.begin();
  end   = object.end();

  while(i--) {
    Update(&glass, begin, end);     // Update glasses and objects
    std::sort(begin, end, Compare); // Sort by key
    ((*begin)->count)++;            // Increment 'count' of the most possible object
    
  #if SHOW
    system("clear");
    Info(&glass);
    Info(begin, end);
    std::cout << std::endl << std::endl;
  #endif
    loop_rate.sleep();
  }

  // Decide by majority
  std::sort(begin, end, Compare2);
  j = 1;
  for (it = begin; it != end; ++it) {
      std::cout << j++ << " ";
      std::cout << (*it)->name << std::endl;
  }
  std::cout << std::endl;
  it = begin;

  // Publish
  msg.name = (*it)->name;
  msg.name = (*it)->objectID;
  display_pub.publish(msg);

  // Store the result to the response
  res.id_out = (*it)->objectID;
  res.name   = (*it)->name;
  res.x = (*it)->pos.x + (*it)->offset_pos.x;
  res.y = (*it)->pos.y + (*it)->offset_pos.y;
  res.z = (*it)->pos.z + 0.15;//2*((*it)->offset_pos.z);

  // Reset the number of 'count'
  for (it = begin; it != end; ++it) { (*it)->count =0; }

  return true;
}

//------------------------------------------------------------------------------
// Main
int main(int argc, char **argv) {
  ros::init(argc, argv, "object_selector");

  if (argc < 2) {
    ROS_ERROR("No argument has found");
    ROS_INFO("Set the mode to 1");
    mode = 1;
  } else {
    mode = atoi(argv[1]); // Slect mode to toggle sort key
  }

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("object_decision",SortList);

  ROS_INFO("Ready to receive the ID list");
  ros::spin();

  return 0;
}
