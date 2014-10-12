#ifndef TMS_RP_RP_H_
#define TMS_RP_RP_H_

#include <tms_rp_bar.h>

#include <tms_msg_rp/rp_cmd.h>
#include <tms_msg_rp/rp_arrow.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_rp/rps_goal_planning.h>
#include <tms_msg_rp/rps_joint_angle.h>
#include <tms_msg_rp/rps_cnoid_grasp_obj_planning.h>
#include <tms_msg_rc/tms_rc_pmove.h>
#include <tms_msg_rc/katana_pos_array.h>
#include <tms_msg_rc/rc_robot_control.h>
#include <tms_msg_rs/rs_home_appliances.h>

namespace tms_rp {

class TmsRpSubtask : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
	TmsRpSubtask();
	static TmsRpSubtask* instance();
	virtual ~TmsRpSubtask();

	bool get_robot_pos(bool type, int robot_id, std::string& robot_name, tms_msg_rp::rps_voronoi_path_planning& rp_srv);
	bool subtask(tms_msg_rp::rp_cmd::Request &req,tms_msg_rp::rp_cmd::Response &res);
	bool sp5_control(bool type, int unit, int cmd, int arg_size, double* arg);
	bool kxp_control(bool type, int unit, int cmd, int arg_size, double* arg);

 private:
	uint32_t sid;

	ros::ServiceServer rp_subtask_server;
	ros::ServiceClient get_data_client;
	ros::ServiceClient sp5_control_client;
	ros::ServiceClient sp5_virtual_control_client;
	ros::ServiceClient kxp_virtual_control_client;
	ros::ServiceClient kxp_mbase_client;
	ros::ServiceClient kobuki_virtual_control_client;
	ros::ServiceClient voronoi_path_planning_client;
	ros::ServiceClient give_obj_client;
	ros::ServiceClient refrigerator_client;

	std::ostream& os;
	grasp::TmsRpController& tac;
};

class TmsRpView : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
	TmsRpView();
	static TmsRpView* instance();
	virtual ~TmsRpView();

	ros::ServiceServer rp_blink_arrow_server;

	bool blink_arrow(tms_msg_rp::rp_arrow::Request &req,
            tms_msg_rp::rp_arrow::Response &res);

 private:
	Matrix3d mat0;
	grasp::TmsRpController& tac;
};
}

#endif /* TMS_RP_RP_H_ */
