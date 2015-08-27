#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Bool.h"

#include "mongodb_store_msgs/StringPairList.h"

#include <sstream>
#include <utility>
#include <vector>

#include <boost/foreach.hpp>

using namespace std_msgs;
using namespace geometry_msgs;
using namespace mongodb_store;
using namespace mongodb_store_msgs;
using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_multi_event_log");
	ros::NodeHandle nh;


	// let's say we have a couple of things that we need to store together
    // these could be some sensor data, results of processing etc.
    Pose pose;
    pose.position.z = 666;

    Point point;
    Quaternion quaternion;
    // note that everything that is pass to the message_store must be a ros message type
    // therefore use std_msg types for standard data types like float, int, bool, string etc
    Bool result;
    result.data = true;

	// we will store our results in a separate collection
	MessageStoreProxy messageStore(nh, "pose_process_results");

	vector< pair<string, string> > stored;
	// now add objects and store ids with the addition of type strings for safety. The types are not necessary unless you want to do some kind of reflection on this data later.
	stored.push_back( make_pair(get_ros_type(pose), messageStore.insert(pose)) );
	stored.push_back( make_pair(get_ros_type(point), messageStore.insert(point)) );
	stored.push_back( make_pair(get_ros_type(quaternion), messageStore.insert(quaternion)) );
	stored.push_back( make_pair(get_ros_type(result), messageStore.insert(result)) );

	// add these things to a single structure for storing in the db
	StringPairList spl;

	for (vector< pair<string, string> >::iterator i = stored.begin(); i != stored.end(); ++i)
	{
		spl.pairs.push_back(mongodb_store::makePair(i->first, i->second));
	}		
        // meta['description'] = "this wasn't great"    
        // meta['result_time'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())


	// and add some descriptive information
	mongo::BSONObjBuilder metaBuilder;
	metaBuilder.append("description", "this wasn\'t great, was it?");
	metaBuilder.append("result_time", mongo::Date_t(ros::Time::now().toSec() * 1000));

	// and store
    messageStore.insert(spl, metaBuilder.obj());

    // now let's get all our logged data back, along with meta information
	vector< pair<boost::shared_ptr<StringPairList>, mongo::BSONObj> > results;
	
	messageStore.query<StringPairList>(results);
       

	for (vector< pair<boost::shared_ptr<StringPairList>, mongo::BSONObj> >::iterator message_and_meta = results.begin(); message_and_meta != results.end(); ++message_and_meta)
	{
			
		// Hmmm... this code is ugly

		const boost::shared_ptr<StringPairList> & message = message_and_meta->first;
		const mongo::BSONObj & meta = message_and_meta->second;

  		if (meta.hasField("description")) {
  			cout << meta["description"].toString() << endl;
  		} 

  		// comment this out as toTimeT() doesn't seem to work on Ubuntu...

		// char buff[20];

		// mongo::Date_t result_time;
		// meta["result_time"].Val(result_time);
		// time_t t(result_time.toTimeT());
		// strftime(buff, 20, "%Y-%m-%d %H:%M:%S", localtime(&t));

		// cout << "result time (local time from rostime): " << buff << endl;

		// mongo::Date_t inserted_at;
		// meta["inserted_at"].Val(inserted_at);
		// t = inserted_at.toTimeT();
		// strftime(buff, 20, "%Y-%m-%d %H:%M:%S", localtime(&t));
		
		// cout << "inserted at (local time from rostime): " << buff << endl;

		// get the objects back
		pair<boost::shared_ptr<Pose>, mongo::BSONObj> storedPose(messageStore.queryID<Pose>(message->pairs[0].second));
		pair<boost::shared_ptr<Point>, mongo::BSONObj> storedPoint(messageStore.queryID<Point>(message->pairs[1].second));
		pair<boost::shared_ptr<Quaternion>, mongo::BSONObj> storedQuaternion(messageStore.queryID<Quaternion>(message->pairs[2].second));		
		pair<boost::shared_ptr<Bool>, mongo::BSONObj> storedBool(messageStore.queryID<Bool>(message->pairs[3].second));

	}       

	return 0;
}


