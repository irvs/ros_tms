/***************************************************************************
 *  mongodb_log_tf.cpp - MongoDB Logger for /tf
 *
 *  Created: Wed Dec 8 17:00:25 2010 -0500
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *             2010  Carnegie Mellon University
 *             2010  Intel Labs Pittsburgh
 *             2014  Jan Winkler <winkler@cs.uni-bremen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

// System
#include <list>
#include <string>

// ROS
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/LinearMath/Quaternion.h>

// MongoDB
#include <mongo/client/dbclient.h>
#include <mongodb_store/util.h>


using namespace mongo;


typedef struct {
  geometry_msgs::TransformStamped tsTransform;
} PoseStampedMemoryEntry;

float fVectorialDistanceThreshold;
float fAngularDistanceThreshold;
float fTimeDistanceThreshold;
list<PoseStampedMemoryEntry> lstPoseStampedMemory;
bool bAlwaysLog;

DBClientConnection *mongodb_conn;
std::string collection;
std::string topic;

unsigned int in_counter;
unsigned int out_counter;
unsigned int qsize;
unsigned int drop_counter;

static pthread_mutex_t in_counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t out_counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t drop_counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t qsize_mutex = PTHREAD_MUTEX_INITIALIZER;

bool shouldLogTransform(std::vector<geometry_msgs::TransformStamped>::const_iterator t) {
  if(bAlwaysLog) {
    // When this flag is set, always return true immediately (and
    // don't keep track of logged transforms).
    return true;
  }
  
  string strMsgFrame = t->header.frame_id;
  string strMsgChild = t->child_frame_id;
  bool bFound = false;
  
  for(list<PoseStampedMemoryEntry>::iterator itEntry = lstPoseStampedMemory.begin();
      itEntry != lstPoseStampedMemory.end();
      itEntry++) {
    string strEntryFrame = (*itEntry).tsTransform.header.frame_id;
    string strEntryChild = (*itEntry).tsTransform.child_frame_id;
    
    // Is this the same transform as in tfMsg?
    if((strEntryFrame == strMsgFrame && strEntryChild == strMsgChild) ||
       (strEntryFrame == strMsgChild && strEntryChild == strMsgFrame)) {
      // Yes, it is. Check vectorial and angular distance.
      bFound = true;
      
      float fVectorialDistance = sqrt(((t->transform.translation.x - (*itEntry).tsTransform.transform.translation.x) *
				       (t->transform.translation.x - (*itEntry).tsTransform.transform.translation.x)) +
				      ((t->transform.translation.y - (*itEntry).tsTransform.transform.translation.y) *
				       (t->transform.translation.y - (*itEntry).tsTransform.transform.translation.y)) +
				      ((t->transform.translation.z - (*itEntry).tsTransform.transform.translation.z) *
				       (t->transform.translation.z - (*itEntry).tsTransform.transform.translation.z)));
      
      tf::Quaternion q1(t->transform.rotation.x, t->transform.rotation.y, t->transform.rotation.z, t->transform.rotation.w);
      tf::Quaternion q2((*itEntry).tsTransform.transform.rotation.x,
			(*itEntry).tsTransform.transform.rotation.y,
			(*itEntry).tsTransform.transform.rotation.z,
			(*itEntry).tsTransform.transform.rotation.w);
      
      float fAngularDistance = 2.0 * fabs(q1.angle(q2));
      
      float fTimeDistance = (fabs((t->header.stamp.sec * 1000.0 + t->header.stamp.nsec / 1000000.0) -
				  ((*itEntry).tsTransform.header.stamp.sec * 1000.0 + (*itEntry).tsTransform.header.stamp.nsec / 1000000.0)) / 1000.0);
      
      if(((fVectorialDistance > fVectorialDistanceThreshold) ||
	  (fAngularDistance > fAngularDistanceThreshold) ||
	  (fTimeDistanceThreshold > 0 &&
	   (fTimeDistance > fTimeDistanceThreshold)))) {
	// Requirements met, this transform should be logged and the
	// stored entry renewed.
	(*itEntry).tsTransform = *t;
	
	return true;
      }
    }
  }
  
  if(!bFound) {
    // This transform is new, so log it.
    PoseStampedMemoryEntry psEntry;
    psEntry.tsTransform = *t;
    lstPoseStampedMemory.push_back(psEntry);
    
    return true;
  }
  
  return false;
}

void msg_callback(const tf::tfMessage::ConstPtr& msg) {
  std::vector<BSONObj> transforms;
  
  const tf::tfMessage& msg_in = *msg;
  bool bDidLogTransforms = false;
  
  std::vector<geometry_msgs::TransformStamped>::const_iterator t;
  for (t = msg_in.transforms.begin(); t != msg_in.transforms.end(); ++t) {
    if(shouldLogTransform(t)) {
      bDidLogTransforms = true;
      
      Date_t stamp = t->header.stamp.sec * 1000.0 + t->header.stamp.nsec / 1000000.0;
      
      BSONObjBuilder transform_stamped;
      BSONObjBuilder transform;
      transform_stamped.append("header", BSON(   "seq" << t->header.seq
					      << "stamp" << stamp
					      << "frame_id" << t->header.frame_id));
      transform_stamped.append("child_frame_id", t->child_frame_id);
      transform.append("translation", BSON(   "x" << t->transform.translation.x
					   << "y" << t->transform.translation.y
					   << "z" << t->transform.translation.z));
      transform.append("rotation", BSON(   "x" << t->transform.rotation.x
					<< "y" << t->transform.rotation.y
					<< "z" << t->transform.rotation.z
					<< "w" << t->transform.rotation.w));
      transform_stamped.append("transform", transform.obj());
      transforms.push_back(transform_stamped.obj());
    }
  }
  
  if(bDidLogTransforms) {
    mongodb_conn->insert(collection, BSON("transforms" << transforms <<
					  "__recorded" << Date_t(time(NULL) * 1000) <<
					  "__topic" << topic));
    
    // If we'd get access to the message queue this could be more useful
    // https://code.ros.org/trac/ros/ticket/744
    pthread_mutex_lock(&in_counter_mutex);
    ++in_counter;
    pthread_mutex_unlock(&in_counter_mutex);
    pthread_mutex_lock(&out_counter_mutex);
    ++out_counter;
    pthread_mutex_unlock(&out_counter_mutex);
  }
}

void print_count(const ros::TimerEvent &te) {
  unsigned int l_in_counter, l_out_counter, l_drop_counter, l_qsize;
  
  pthread_mutex_lock(&in_counter_mutex);
  l_in_counter = in_counter; in_counter = 0;
  pthread_mutex_unlock(&in_counter_mutex);
  
  pthread_mutex_lock(&out_counter_mutex);
  l_out_counter = out_counter; out_counter = 0;
  pthread_mutex_unlock(&out_counter_mutex);
  
  pthread_mutex_lock(&drop_counter_mutex);
  l_drop_counter = drop_counter; drop_counter = 0;
  pthread_mutex_unlock(&drop_counter_mutex);
  
  pthread_mutex_lock(&qsize_mutex);
  l_qsize = qsize; qsize = 0;
  pthread_mutex_unlock(&qsize_mutex);
  
  printf("%u:%u:%u:%u\n", l_in_counter, l_out_counter, l_drop_counter, l_qsize);
  fflush(stdout);
}

int main(int argc, char **argv) {
  std::string mongodb = "localhost", nodename = "";
  collection = topic = "";
  
  in_counter = out_counter = drop_counter = qsize = 0;
  
  fVectorialDistanceThreshold = 0.100; // Distance threshold in terms
				       // of vector distance between
				       // an old and a new pose of the
				       // same transform before it
				       // gets logged again
  fAngularDistanceThreshold = 0.100; // Same for angular distance
  fTimeDistanceThreshold = 1.0; // And same for timely distance (in seconds)
  bAlwaysLog = true;
  
  int c;
  while ((c = getopt(argc, argv, "t:m:n:c:ak:l:g:")) != -1) {
    if ((c == '?') || (c == ':')) {
      printf("Usage: %s -t topic -m mongodb -n nodename -c collection -k vectorial-threshold -l angular-threshold -g time-threshold -a\n", argv[0]);
      exit(-1);
    } else if (c == 't') {
      topic = optarg;
    } else if (c == 'm') {
      mongodb = optarg;
    } else if (c == 'n') {
      nodename = optarg;
    } else if (c == 'c') {
      collection = optarg;
    } else if (c == 'a') {
      bAlwaysLog = false;
    } else if (c == 'k') {
      sscanf(optarg, "%f", &fVectorialDistanceThreshold);
    } else if (c == 'l') {
      sscanf(optarg, "%f", &fAngularDistanceThreshold);
    } else if (c == 'g') {
      sscanf(optarg, "%f", &fTimeDistanceThreshold);
    }
  }
  
  if (topic == "") {
    printf("No topic given.\n");
    exit(-2);
  } else if (nodename == "") {
    printf("No node name given.\n");
    exit(-2);
  }
  
  ros::init(argc, argv, nodename);
  ros::NodeHandle n;
  
  std::string errmsg;
  mongodb_conn = new DBClientConnection(/* auto reconnect*/ true);
  if (! mongodb_conn->connect(mongodb, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
    return -1;
  }

  ros::Subscriber sub = n.subscribe<tf::tfMessage>(topic, 1000, msg_callback);
  ros::Timer count_print_timer = n.createTimer(ros::Duration(5, 0), print_count);

  ros::spin();

  delete mongodb_conn;

  return 0;
}
