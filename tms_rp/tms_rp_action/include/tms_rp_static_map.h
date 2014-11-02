#ifndef TMS_RP_STATIC_MAP_H
#define TMS_RP_STATIC_MAP_H

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

class TmsRpStaticMap : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
  TmsRpStaticMap();
  static TmsRpStaticMap* instance();
  virtual ~TmsRpStaticMap();
  void mapPublish();
  bool setVoronoiLine(vector<vector<CollisionMapData> >& map, string& message);

 private:
  std::ostream& os_;
  grasp::TmsRpController& tac_;
  uint32_t sid_;

  ros::Publisher pp_map_pub_;
  vector<vector<CollisionMapData> > collision_map_;
  tms_msg_rp::rps_map_full pub_map_;
  string result_msg_;

  double x_llimit_, x_ulimit_, y_llimit_, y_ulimit_, cell_size_; //M

  bool initCollisionMap(vector<vector<CollisionMapData> >& map);
  bool calcDistFromObj(vector<vector<CollisionMapData> >& map, string& message);
  void convertMap(vector<vector<CollisionMapData> > map, tms_msg_rp::rps_map_full& pp_map);
};
}

#endif // TMS_RP_STATIC_MAP_H
