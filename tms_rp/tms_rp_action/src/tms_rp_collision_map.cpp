#include <tms_rp_collision_map.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

//------------------------------------------------------------------------------
CollisionTarget::CollisionTarget(BodyItemPtr bodyItem){
  bodyItem_collision_target_ = bodyItem;
  base_ = bodyItem_collision_target_->body()->link(0);
}

//------------------------------------------------------------------------------
TmsRpCollisionMap* TmsRpCollisionMap::instance()
{
  static TmsRpCollisionMap* instance = new TmsRpCollisionMap();
  return instance;
}

//------------------------------------------------------------------------------
TmsRpCollisionMap::TmsRpCollisionMap() {
  collision_target_ = NULL;
}

//------------------------------------------------------------------------------
TmsRpCollisionMap::~TmsRpCollisionMap()
{
}

//------------------------------------------------------------------------------
void TmsRpCollisionMap::setCollisionTarget(BodyItemPtr bodyItem){
  cout << "setCollisionTarget" << endl;
  collision_target_ = new CollisionTarget(bodyItem);
}

//------------------------------------------------------------------------------
void TmsRpCollisionMap::initialCollision(){
  if(collision_target_){
    collision_map_pairs_.clear();

    for(unsigned int j=0;j<collision_target_->bodyItem_collision_target_->body()->numLinks();j++){
      for( list<BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it !=PlanBase::instance()->bodyItemEnv.end(); it++){
        for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
          collision_map_pairs_.push_back(make_shared<ColdetLinkPair>(collision_target_->bodyItem_collision_target_->body(),collision_target_->bodyItem_collision_target_->body()->link(j), (*it)->body(),(*it)->body()->link(i)));
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
bool TmsRpCollisionMap::makeCollisionMap(vector<vector<int> >& out_collision_map){
  if(cell_size_<=0.0){
    cout<<"Error : please set cell_size_ more than 0"<<endl;
    return false;
  }
  out_collision_map.clear();
  initialCollision();

  vector<int>  temp_line;
  temp_line.clear();

  for(double x = x_llimit_;x<=x_ulimit_+1.0e-10;x=x+cell_size_){
    temp_line.clear();
    for(double y = y_llimit_;y<=y_ulimit_+1.0e-10;y=y+cell_size_){
      collisionTargetBase()->p()(0) = x;
      collisionTargetBase()->p()(1) = y;
      collision_target_->bodyItem_collision_target_->body()->calcForwardKinematics();
      collision_target_->bodyItem_collision_target_->notifyKinematicStateChange();
      MessageView::mainInstance()->flush();

      temp_line.push_back(isColliding());
    }
    out_collision_map.push_back(temp_line);
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpCollisionMap::isColliding(){
  if(collision_target_){
    for(unsigned int i=0;i<collision_map_pairs_.size();i++){
      ColdetLinkPairPtr testPair = collision_map_pairs_[i];
      testPair->updatePositions();
      bool coll = testPair->checkCollision();
      if(coll){
        return true;
      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
