#ifndef TMS_RP_COLLISION_MAP_H
#define TMS_RP_COLLISION_MAP_H

#include <tms_rp_bar.h>

namespace tms_rp {

class CollisionTarget
{
public:
  CollisionTarget(BodyItemPtr bodyItem);
  const string& name() { return bodyItemCollisionTarget->name(); }

  cnoid::BodyItemPtr bodyItemCollisionTarget;
  cnoid::Link *base;
};

class TmsRpCollisionMap : public boost::signals::trackable
{
public:
  double x_llimit_, x_ulimit_, y_llimit_, y_ulimit_, cell_size_;
  vector<ColdetLinkPairPtr> collisionMapPairs;
  CollisionTarget* collisionTarget;
  cnoid::Link* collisionTargetBase() { return collisionTarget->base; }
  TmsRpCollisionMap();
  static TmsRpCollisionMap* instance();
  virtual ~TmsRpCollisionMap();
  bool makeCollisionMap(vector<vector<int> >& out_collision_map);
  void SetCollisionTarget(BodyItemPtr bodyItem);
private:
  void initialCollision();
  bool isColliding();
};

}

#endif // TMS_RP_COLLISION_MAP_H
