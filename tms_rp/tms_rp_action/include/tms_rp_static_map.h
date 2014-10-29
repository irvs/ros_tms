#ifndef TMS_RP_STATIC_MAP_H
#define TMS_RP_STATIC_MAP_H

#include <tms_rp_bar.h>
#include <tms_msg_rp/rps_map_full.h>

namespace tms_rp {

class CollisionMapData
{
 public:
  bool    object;
  double  dist_from_obj;
  bool    collision;
  bool    voronoi;
  int     thinning_flg;
  double  dist_from_voronoi;
  bool    path;
  double  dist_from_path;
  double  dist_from_goal;
  bool    person_view;
  double  table_height;
};

class TmsRpStaticMap : public cnoid::ToolBar, public boost::signals::trackable
{
 public:
  TmsRpStaticMap();
  static TmsRpStaticMap* instance();
  virtual ~TmsRpStaticMap();
  void mapPublish();

 private:
  uint32_t sid;

  ros::Publisher pp_map_pub;
  vector<vector<CollisionMapData> > map;
  tms_msg_rp::rps_map_full pub_map;
  string msg;

  std::ostream& os;
  grasp::TmsRpController& tac;

  double x_llimit, x_ulimit, y_llimit, y_ulimit, cell_size; //M

  bool initCollisionMap(vector<vector<CollisionMapData> >& map);
  bool setVoronoiLine(vector<vector<CollisionMapData> >& map, string& message);
  bool calcDistFromObj(vector<vector<CollisionMapData> >& map, string& message);
  void convertMap(vector<vector<CollisionMapData> > map, tms_msg_rp::rps_map_full& pp_map);
};
}

#endif // TMS_RP_STATIC_MAP_H
