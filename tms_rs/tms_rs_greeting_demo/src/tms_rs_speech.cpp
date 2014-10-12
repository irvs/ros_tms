/*
 * speech.cpp
 *
 *  Created on: 2014/07/30
 *      Author: hashiguchi
 */

#include <ros/ros.h>
#include <string>
#include <vector>

#include <tms_msg_rc/robot_tts.h>

ros::ServiceClient speech_client;


bool sp5_tts(std::string* sentene, double* timing, int size) {
	tms_msg_rc::robot_tts sp_tts_srv;

	for (int i=0; i<size; i++) {
		sp_tts_srv.request.text = sentene[i];
		if (speech_client.call(sp_tts_srv)) {ROS_INFO("result: %d", sp_tts_srv.response.result);}
		else {
			ROS_ERROR("Failed to call service smartpal5_tts");
			return false;
		}
		ros::Duration(timing[i]).sleep();
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tms_rs_speech");
	ros::NodeHandle n;
	speech_client = n.serviceClient<tms_msg_rc::robot_tts>("smartpal5_tts");

	int size = 4;
	//"Hello, I am Smart pal five."
	std::string s_sentence_list[4] = {"Hello, I am Smart pal five.", "Thank you for coming to our rabo.",
			"Please enjoy watching my demo.", "I will do my best."};
	double timing_list[4] = {9.0, 9.0, 7.0, 3.0};

	sp5_tts(s_sentence_list, timing_list, size);

	return 0;
}
