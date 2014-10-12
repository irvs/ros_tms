/*
 * refrigerator_demo.cpp
 *
 *  Created on: 2014/07/31
 *      Author: hashiguchi
 */

#include <ros/ros.h>
#include <string>
#include <vector>

#include <tms_msg_rc/smartpal_control.h>


#define UNIT_VEHICLE            1
#define UNIT_ARM_R              2
#define UNIT_ARM_L              3
#define UNIT_GRIPPER_R          4
#define UNIT_GRIPPER_L          5
#define UNIT_LUMBA              6

#define CMD_MOVE_ABS            15
#define CMD_MOVE_REL            16

ros::ServiceClient motion_client;

bool sp_control_function(int unit, int cmd, int arg_size, double* arg) {
	tms_msg_rc::smartpal_control sp_control_srv;

	sp_control_srv.request.unit = unit;
	sp_control_srv.request.cmd  = cmd;
	sp_control_srv.request.arg.resize(arg_size);
	for (int i=0; i<sp_control_srv.request.arg.size(); i++) {
		sp_control_srv.request.arg[i] = arg[i];
		//ROS_INFO("arg[%d]=%f", i, sp_control_srv.request.arg[i]);
	}

	if (motion_client.call(sp_control_srv)) ROS_INFO("result: %d", sp_control_srv.response.result);
	else ROS_ERROR("Failed to call service sp5_control");

	return true;
}

bool sp5_control(const double* arg) {

	double arg_vehicle[3] = {0.0, 0.0, arg[0]};
	sp_control_function(UNIT_VEHICLE, CMD_MOVE_REL, 3, arg_vehicle);

	double arg_armR[8] = {arg[3], arg[4], arg[5], arg[6], arg[7], arg[8], arg[9], 30.0};
	sp_control_function(UNIT_ARM_R, CMD_MOVE_ABS, 8, arg_armR);

	double arg_armL[8] = {arg[11], arg[12], arg[13], arg[14], arg[15], arg[16], arg[17], 30.0};
	sp_control_function(UNIT_ARM_L, CMD_MOVE_ABS, 8, arg_armL);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "on_off_refrigerator");
	ros::NodeHandle n;

	motion_client = n.serviceClient<tms_msg_rc::smartpal_control>("sp5_control");

	double point_motion[19] = {-36.0, 0.0, 0.0, 0.0, -24.0, 101.0, 35.8, 65.5, 0.0, -17.7, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double return_motion[19] = {36.0, 0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


	if (sp5_control(point_motion)) ROS_INFO("succeed to call service point_motion.");

	ros::Duration(3.0).sleep();
	//initial
	if (sp5_control(return_motion)){ROS_INFO("succeed to call service return");}

	return 0;
}




