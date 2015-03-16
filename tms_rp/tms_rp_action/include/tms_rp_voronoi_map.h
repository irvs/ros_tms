#ifndef TMS_RP_VORONOI_MAP_H
#define TMS_RP_VORONOI_MAP_H

#include <tms_rp_bar.h>
#include <tms_msg_rp/rps_map_full.h>

namespace tms_rp
{

class CollisionMapData
{
public:
  bool    object_;
  double  dist_from_obj_;
  bool    collision_;
  bool    voronoi_;
  int     thinning_flg_;
  double  dist_from_voronoi_;
  bool    path_;
  double  dist_from_path_;
  double  dist_from_goal_;
  bool    person_view_;
  double  table_height_;
};

class TmsRpVoronoiMap : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
  TmsRpVoronoiMap();
  static TmsRpVoronoiMap* instance();
  virtual ~TmsRpVoronoiMap();

  bool setVoronoiLine(vector<vector<CollisionMapData> >& map, string& message);
  void staticMapPublish();
  void dynamicMapPublish();
  void dynamicMapPublish(tms_msg_ss::tracking_points unknown_moving_object_pos);

 private:
  std::ostream& os_;
  grasp::TmsRpController& trc_;
  uint32_t sid_;

  ros::Publisher static_map_pub_;
  ros::Publisher dynamic_map_pub_;
  vector<vector<CollisionMapData> > collision_map_;
  tms_msg_rp::rps_map_full static_map_;
  tms_msg_rp::rps_map_full dynamic_map_;
  string result_msg_;
  ros::ServiceClient get_data_client_;

  double x_llimit_, x_ulimit_, y_llimit_, y_ulimit_, cell_size_; //M

  bool initCollisionMap(vector<vector<CollisionMapData> >& map);
  bool calcDistFromObj(vector<vector<CollisionMapData> >& map, string& message);
  void convertMap(vector<vector<CollisionMapData> > map, tms_msg_rp::rps_map_full& pp_map);
};
}

#endif // TMS_RP_VORONOI_MAP_H
