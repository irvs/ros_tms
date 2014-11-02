#include <tms_rp_collision_map.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

//------------------------------------------------------------------------------
CollisionTarget::CollisionTarget(BodyItemPtr bodyItem){
  bodyItemCollisionTarget = bodyItem;
  base = bodyItemCollisionTarget->body()->link(0);
}

//------------------------------------------------------------------------------
TmsRpCollisionMap* TmsRpCollisionMap::instance()
{
  static TmsRpCollisionMap* instance = new TmsRpCollisionMap();
  return instance;
}

//------------------------------------------------------------------------------
TmsRpCollisionMap::TmsRpCollisionMap() {
  collisionTarget = NULL;
}

//------------------------------------------------------------------------------
TmsRpCollisionMap::~TmsRpCollisionMap()
{
}

//------------------------------------------------------------------------------
void TmsRpCollisionMap::SetCollisionTarget(BodyItemPtr bodyItem){
  cout << "SetCollisionTarget" << endl;
  collisionTarget = new CollisionTarget(bodyItem);
}

//------------------------------------------------------------------------------
void TmsRpCollisionMap::initialCollision(){
  if(collisionTarget){
    collisionMapPairs.clear();

    for(unsigned int j=0;j<collisionTarget->bodyItemCollisionTarget->body()->numLinks();j++){
      for( list<BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it !=PlanBase::instance()->bodyItemEnv.end(); it++){
        for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
          collisionMapPairs.push_back(make_shared<ColdetLinkPair>(collisionTarget->bodyItemCollisionTarget->body(),collisionTarget->bodyItemCollisionTarget->body()->link(j), (*it)->body(),(*it)->body()->link(i)));
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
      collisionTarget->bodyItemCollisionTarget->body()->calcForwardKinematics();
      collisionTarget->bodyItemCollisionTarget->notifyKinematicStateChange();
      MessageView::mainInstance()->flush();

      temp_line.push_back(isColliding());
    }
    out_collision_map.push_back(temp_line);
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpCollisionMap::isColliding(){
  if(collisionTarget){
    for(unsigned int i=0;i<collisionMapPairs.size();i++){
      ColdetLinkPairPtr testPair = collisionMapPairs[i];
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
