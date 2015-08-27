#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"

#include <boost/foreach.hpp>
#include <gtest/gtest.h>


#include <sstream>

using namespace geometry_msgs;
using namespace mongodb_store;
using namespace std;

// Declare a test
TEST(ROSDatacentre, cppTest)
{
	ros::NodeHandle nh;

	//Create object which does the work for us.
	MessageStoreProxy messageStore(nh, "test_collection");

	//This is the message we want to store
	Pose stored;
	string name("__this__is__a__cpp__test__pose__");
	//Insert something with a name
	string id(messageStore.insertNamed(name, stored));

	vector< boost::shared_ptr<Pose> > results;

	//Get it back, by default get one
	if(messageStore.queryNamed<Pose>(name, results)) {
		EXPECT_EQ(1, results.size());
		EXPECT_EQ(stored.position.x, results[0]->position.x);
        EXPECT_EQ(stored.position.y, results[0]->position.y);
        EXPECT_EQ(stored.position.z, results[0]->position.z);
        EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
        EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
        EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
        EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);
	}
	else {
		ADD_FAILURE() << "Name query didn't find: " << name;
	}

	if(messageStore.queryID<Pose>(id, results)) {

		BOOST_FOREACH( boost::shared_ptr<Pose> p,  results)
		{
			//ROS_INFO_STREAM("Got by ID: " << *p);
		}

	}


	stored.position.x = 999;
	messageStore.updateNamed(name, stored);
	//check the update worked
	results.clear();
	if(messageStore.queryNamed<Pose>(name, results)) {
		EXPECT_EQ(1, results.size());
		EXPECT_EQ(stored.position.x, results[0]->position.x);
        EXPECT_EQ(stored.position.y, results[0]->position.y);
        EXPECT_EQ(stored.position.z, results[0]->position.z);
        EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
        EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
        EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
        EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);
	}
	else {
		ADD_FAILURE() << "Name query didn't find: " << name;
	}


	//check id query
	results.clear();
	if(messageStore.queryID<Pose>(id, results)) {
		EXPECT_EQ(1, results.size());
		EXPECT_EQ(stored.position.x, results[0]->position.x);
        EXPECT_EQ(stored.position.y, results[0]->position.y);
        EXPECT_EQ(stored.position.z, results[0]->position.z);
        EXPECT_EQ(stored.orientation.x, results[0]->orientation.x);
        EXPECT_EQ(stored.orientation.y, results[0]->orientation.y);
        EXPECT_EQ(stored.orientation.z, results[0]->orientation.z);
        EXPECT_EQ(stored.orientation.w, results[0]->orientation.w);
	}
	else {
		ADD_FAILURE() << "ID query didn't find: " << id;
	}

	// try to get it back with an incorrect name, so get None instead	
	std::string wrongName(name + "some garbage addition");
	results.clear();
	if(messageStore.queryNamed<Pose>(wrongName, results)) {
		ADD_FAILURE() << "Name query shouldn't have found: " << wrongName;
	}
	else {
		EXPECT_EQ(0, results.size());
	}


	// get all poses, should be at least one
	results.clear();
	EXPECT_TRUE(messageStore.query<Pose>(results));
	EXPECT_GT(results.size(), 0);


	// Now delete the thing we added before
	messageStore.deleteID(id);

	results.clear();
	if(messageStore.queryID<Pose>(id, results)) {
		ADD_FAILURE() << "ID query shouldn't have found " << id << " after deletion";
	}
	else {
		EXPECT_EQ(0, results.size());
	}

	results.clear();
	if(messageStore.queryNamed<Pose>(name, results)) {
		ADD_FAILURE() << "Name query shouldn't have found " << name << " after deletion";
	}
	else {
		EXPECT_EQ(0, results.size());
	}


}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "example_mongodb_store_cpp_client");
	return RUN_ALL_TESTS();
}


