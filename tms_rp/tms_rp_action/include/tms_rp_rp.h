#ifndef TMS_RP_RP_H_
#define TMS_RP_RP_H_

#include <tms_rp_bar.h>

#include <tms_msg_ts/ts_state_control.h>
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
#include <tms_msg_ss/ods_person_dt.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/MotorPower.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <stdlib.h>
#include <math.h>

namespace tms_rp {

class TmsRpSubtask : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
	TmsRpSubtask();
	static TmsRpSubtask* instance();
	virtual ~TmsRpSubtask();

	void send_rc_exception(int error_type);
	bool get_robot_pos(bool type, int robot_id, std::string& robot_name, tms_msg_rp::rps_voronoi_path_planning& rp_srv);
	bool subtask(tms_msg_rp::rp_cmd::Request &req,tms_msg_rp::rp_cmd::Response &res);
	bool sp5_control(bool type, int unit, int cmd, int arg_size, double* arg);
	bool kxp_control(bool type, int unit, int cmd, int arg_size, double* arg);
	void sensingCallback(const tms_msg_ss::ods_person_dt::ConstPtr& msg);

 private:
	uint32_t sid_;
	struct SubtaskData {
		bool type;
		int robot_id;
		int arg_type;
		std::vector<double> v_arg;
	};

	// for thread
	bool move(SubtaskData sd); // 9001
	bool grasp(SubtaskData sd); // 9002
	bool give(SubtaskData sd); // 9003
	bool open_ref(void);  // 9004
	bool close_ref(void); // 9005
	bool random_move(void); // 9006
	bool sensing(void); // 9007

	ros::ServiceServer rp_subtask_server;
	ros::ServiceClient get_data_client_;
	ros::ServiceClient sp5_control_client_;
	ros::ServiceClient sp5_virtual_control_client;
	ros::ServiceClient kxp_virtual_control_client;
	ros::ServiceClient kxp_mbase_client;
	ros::ServiceClient kobuki_virtual_control_client;
	ros::ServiceClient mkun_virtual_control_client;
	ros::ServiceClient mkun_control_client;
	ros::ServiceClient voronoi_path_planning_client_;
	ros::ServiceClient give_obj_client;
	ros::ServiceClient refrigerator_client;
	ros::ServiceClient state_client;

	ros::Publisher kobuki_sound;
	ros::Publisher kobuki_motorpower;
	ros::Subscriber sensing_sub;

	std::ostream& os_;
	grasp::TmsRpController& trc_;
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
	Matrix3d mat0_;
	grasp::TmsRpController& trc_;
};
}

#endif /* TMS_RP_RP_H_ */
