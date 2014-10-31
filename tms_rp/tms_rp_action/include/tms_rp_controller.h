#ifndef _TMS_ACTION_CONTROLLER_H_INCLUDED
#define _TMS_ACTION_CONTROLLER_H_INCLUDED

#include <Grasp/PlanBase.h>
#include <Grasp/GraspController.h>
#include <Grasp/GraspBar.h>
#include <tms_msg_db/tmsdb_get_objects_info.h>

#define UNIT_ALL                0
#define UNIT_VEHICLE            1
#define UNIT_ARM_R              2
#define UNIT_ARM_L              3
#define UNIT_GRIPPER_R          4
#define UNIT_GRIPPER_L          5
#define UNIT_LUMBA              6
#define UNIT_CC                 7

#define CMD_CLEARALARM          0
#define CMD_SETPOWER            1
#define CMD_SETSERVO            2
#define CMD_PAUSE               3
#define CMD_RESUME              4
#define CMD_ABORT               5
#define CMD_STOP                6
#define CMD_GETSTATE            7
#define CMD_GETPOSE             8
#define CMD_SYNC_OBJ            8
#define CMD_CALC_BACKGROUND     9
#define CMD_MOVE_ABS            15
#define CMD_MOVE_REL            16

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

namespace grasp {

class pathInfo{
	public:
	std::vector<double> pos;
	cnoid::Vector3 robotPos;
	cnoid::Matrix3 robotOri;
	int state;
	bool moveJoints;
	bool moveBase;
	cnoid::VectorXd joints;
};

class planGraspPath{
	public:
		enum STATE { APPROACH , CLOSING_GRIPPER, UP_HAND, BACKAWAY , DOWN_HAND, OPENING_GRIPPER };
		enum PLANNING_MODE { GRAPSP_PLAN, RELEASE_PLAN, PATH_PLAN, PATH_PLAN_WITH_GRASPED_OBJECT };
		enum BOOL_PARAMETER { TOLERANCE_MODE, PLAN_BASE_MODE, PLAN_JOINT_MODE, SYNC_JOINT_BASE_MODE};
};

class TmsRpController{
	public:
    TmsRpController();
//		~TmsRpController(){}
    static TmsRpController* instance();

    bool setTolerance(double setTolerance);

    bool graspPathPlanStart_(
    		int mode, std::vector<double> begin, std::vector<double> end,
    		std::string robotId, std::string objectTagId, double resolution,
    		std::vector<pathInfo>* trajectory, int* state,
    		std::vector<double>* obj_pos, std::vector<double>* obj_rot);

    bool graspPathPlanStart(
    		int mode, std::vector<double> begin, std::vector<double> end,
    		std::string robotId, std::string objectTagId, double resolution,
    		std::vector<pathInfo>* trajectory, int* state);

    bool pathPlanStart(
    		int mode, std::vector<double> begin, std::vector<double> end,
    		std::string robotId, std::string objectTagId, double resolution,
			std::vector<pathInfo>* trajectory, int* state);

    bool releasePathPlanStart(
    		int mode, std::vector<double> begin, std::vector<double> end,
			std::string robotId, std::string objectTagId, double resolution,
			std::vector<pathInfo>* trajectory, int* state);

    bool setRobotPosture(int mode, std::string robotId, std::vector<double> begin,int* state);
    bool setRobotState(int mode,  std::string robotId, cnoid::Vector3 pos, cnoid::Matrix3 ori, std::vector<double> begin, int* state, cnoid::Vector3 graspPos, cnoid::Matrix3 graspOri);

    bool selectReleasePattern(cnoid::Vector3 objVisPos, cnoid::Matrix3 objVisRot);
	
    bool createRobotRecord(int objId, std::string tagId);
    bool createRecord(int objId, std::string tagId);

    bool deleteRecord(std::string tagId);
    bool appear(std::string tagId);
    bool appear(std::string tagId, std::string robId);
    bool disappear(std::string tagId);
    bool disappear(std::string tagId, std::string robId);
    bool setPos(std::string tagId, cnoid::Vector3 pos, cnoid::Matrix3 ori);
    bool set_all_Pos(std::string tagId, cnoid::Vector3 pos, cnoid::Matrix3 ori, std::vector<double> begin);
    bool getPos(std::string tagId, cnoid::Vector3 *pos, cnoid::Matrix3 *ori);
    bool setBoolParameter(int mode,bool onoff);

    std::string objectBasePath;
    std::map <int, std::string> objId2File;
    std::map <int, std::string> objId2PrePlan;
    std::map <std::string, std::string> objTag2PrePlan;

    std::map <std::string,cnoid::BodyItemPtr>& objTag2Item(){
      return PlanBase::instance()->objTag2Item;
    }
    std::map <std::string,ArmFingers*>& robTag2Arm(){
      return PlanBase::instance()->robTag2Arm;
    }

    void setTrajectoryPlanWholeDOF();

    bool isToleranceMode;
    bool isPlanBaseMode;
    bool isPlanJointMode;
    bool isSyncJointBaseMode;

    cnoid::Vector3 objectPalmPos;
    cnoid::Matrix3 objectPalmRot;


    private:
    std::ostream& os;
    bool setOutputData(std::vector<pathInfo>* trajectory, const int resolution);
};


}

#endif
