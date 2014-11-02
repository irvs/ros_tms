#ifndef TMS_RP_COLLISION_MAP_H
#define TMS_RP_COLLISION_MAP_H

#include <tms_rp_bar.h>

namespace tms_rp {

class CollisionTarget
{
public:
  cnoid::BodyItemPtr bodyItem_collision_target_;
  cnoid::Link *base_;

  CollisionTarget(BodyItemPtr bodyItem);
  const string& name() { return bodyItem_collision_target_->name(); }
};

class TmsRpCollisionMap : public boost::signals::trackable
{
public:
  double x_llimit_, x_ulimit_, y_llimit_, y_ulimit_, cell_size_;
  CollisionTarget* collision_target_;
  vector<ColdetLinkPairPtr> collision_map_pairs_;

  TmsRpCollisionMap();
  static TmsRpCollisionMap* instance();
  virtual ~TmsRpCollisionMap();

  cnoid::Link* collisionTargetBase() { return collision_target_->base_; }

  bool makeCollisionMap(vector<vector<int> >& out_collision_map);
  void setCollisionTarget(BodyItemPtr bodyItem);

private:
  void initialCollision();
  bool isColliding();
};

}

#endif // TMS_RP_COLLISION_MAP_H
