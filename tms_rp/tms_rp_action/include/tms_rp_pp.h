#ifndef TMS_RP_PP_H
#define TMS_RP_PP_H

#include <tms_rp_bar.h>
#include <tms_rp_voronoi_map.h>

#include <tms_msg_rp/rps_map_full.h>
#include <tms_msg_rp/rps_voronoi_path_planning.h>
#include <tms_msg_db/tmsdb_get_robots_info.h>
#include <tms_msg_db/tmsdb_get_movable_furnitures_info.h>
#include <tms_msg_rp/rps_map_full.h>

namespace tms_rp {

class TmsRpPathPlanning : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
  TmsRpPathPlanning();
  static TmsRpPathPlanning* instance();
  virtual ~TmsRpPathPlanning();

  vector<vector<CollisionMapData> > static_map_;

 private:
  std::ostream& os_;
  grasp::TmsRpController& tac_;

  ros::Subscriber static_map_sub_;
  ros::ServiceServer service_voronoi_path_;
  ros::Publisher robot_path_pub_;

  double x_llimit_, x_ulimit_, y_llimit_, y_ulimit_, cell_size_; // Meter

  const double kSmartPal5CollisionThreshold_; //mm
  const double kSmartPal4CollisionThreshold_; //mm
  const double kKobukiCollisionThreshold_;    //mm
  const double kKKPCollisionThreshold_;       //mm
  const double kRobotControlWagonDist_;       //mm
  const double kSmoothVoronoiPathThreshold_;  //mm
  const double kPushWagonPathThreshold_;      //mm

  void getStaticMap(const tms_msg_rp::rps_map_full::ConstPtr& original_map);
  bool voronoiPathPlanner(tms_msg_rp::rps_voronoi_path_planning::Request& req, tms_msg_rp::rps_voronoi_path_planning::Response& res);

  double getRobotCollisionThreshold(int robot_id);
  bool setCollisionArea(vector<vector<CollisionMapData> >& map, double threshold, string& message);
  bool setVoronoiLine(vector<vector<CollisionMapData> >& map, string& message);
  bool calcDistFromVoronoi(vector<vector<CollisionMapData> >& map, string& message);
  bool connectToVoronoi(vector<vector<CollisionMapData> >& map, vector<double> connect_point, string& message);
  bool calcDistFromGoal(vector<vector<CollisionMapData> >& map, vector<double> goal_point, string& message);
  bool calcVoronoiPath(vector<vector<CollisionMapData> >& map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path, string& message);
  bool smoothVoronoiPath(vector<vector<CollisionMapData> >& map, vector<double> start, vector<double> goal, vector<vector<double> > in_path, vector<vector<double> >& out_path, double threshold);

};
}

#endif // TMS_RP_PP_H
