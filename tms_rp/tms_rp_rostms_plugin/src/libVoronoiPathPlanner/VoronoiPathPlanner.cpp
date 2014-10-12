#include "VoronoiPathPlanner.h"
#include <cnoid/MessageView>

using namespace std;
using namespace cnoid;
using namespace grasp;

VoronoiPathPlanner::VoronoiPathPlanner()  :   os (MessageView::mainInstance()->cout() ){
  collisionTarget = NULL;
  start_pos.resize(3);
  goal_pos.resize(3);
}

VoronoiPathPlanner::~VoronoiPathPlanner() {
}

VoronoiPathPlanner* VoronoiPathPlanner::instance(VoronoiPathPlanner *gc) {
  static VoronoiPathPlanner* instance = (gc) ? gc : new VoronoiPathPlanner();
  if(gc) instance = gc;
  return instance;
}

CollisionTarget::CollisionTarget(BodyItemPtr bodyItem){
  bodyItemCollisionTarget = bodyItem;
  base = bodyItemCollisionTarget->body()->link(0);
}

void VoronoiPathPlanner::initialCollision(){
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

bool VoronoiPathPlanner::isColliding(){
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

void VoronoiPathPlanner::SetCollisionTarget(BodyItemPtr bodyItem){
  collisionTarget = new CollisionTarget(bodyItem);
}
	
void VoronoiPathPlanner::MapOutput(vector<vector<CollisionMapData> > Map){
  //QString DirName = QDir::currentPath() + "/extplugin/rostmsPlugin/libVoronoiPathPlanner/CollisionMap/debug/";
  QString DirName = "/home/rt/catkin_ws/src/ros_tms/tms_rp/tms_rp_rostms_plugin/src/libVoronoiPathPlanner/CollisionMap/debug/";
	QString fileName_obj = DirName + "obj.csv";
	QString fileName_dist_from_obj = DirName + "dist_from_obj.csv";
	QString fileName_collision = DirName + "collision.csv";
	QString fileName_voronoi = DirName + "voronoi.csv";
	QString fileName_dist_from_voronoi = DirName + "dist_from_voronoi.csv";
	QString fileName_dist_from_goal = DirName + "dist_from_goal.csv";
   
  QFile file_obj(fileName_obj);
  QFile file_dist_from_obj(fileName_dist_from_obj);
  QFile file_collision(fileName_collision);
  QFile file_voronoi(fileName_voronoi);
  QFile file_dist_from_voronoi(fileName_dist_from_voronoi);
  QFile file_dist_from_goal(fileName_dist_from_goal);
  file_obj.open(QIODevice::WriteOnly);
  file_dist_from_obj.open(QIODevice::WriteOnly);
  file_collision.open(QIODevice::WriteOnly);
  file_voronoi.open(QIODevice::WriteOnly);
  file_dist_from_voronoi.open(QIODevice::WriteOnly);
  file_dist_from_goal.open(QIODevice::WriteOnly);
  
  char temp[256];
  
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      sprintf(temp, "%d,", Map[x][y].object);
      file_obj.write(temp);
      sprintf(temp, "%f,", Map[x][y].dist_from_obj);
      file_dist_from_obj.write(temp);
      sprintf(temp, "%d,", Map[x][y].collision);
      file_collision.write(temp);
      sprintf(temp, "%d,", Map[x][y].voronoi);
      file_voronoi.write(temp);
      sprintf(temp, "%f,", Map[x][y].dist_from_voronoi);
      file_dist_from_voronoi.write(temp);
      sprintf(temp, "%f,", Map[x][y].dist_from_goal);
      file_dist_from_goal.write(temp);
    }
    file_obj.write("\n");
    file_dist_from_obj.write("\n");
    file_collision.write("\n");
    file_voronoi.write("\n");
    file_dist_from_voronoi.write("\n");
    file_dist_from_goal.write("\n");
  }
  
  file_obj.close();
  file_dist_from_obj.close();
  file_collision.close();
  file_voronoi.close();
  file_dist_from_voronoi.close();
  file_dist_from_goal.close();
}

bool VoronoiPathPlanner::calcDistFromObj(vector<vector<CollisionMapData> >& Map){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }
  
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      if(Map[x][y].object)
        Map[x][y].dist_from_obj = 0.0;
      else
        Map[x][y].dist_from_obj = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }
  
  for(int x=1;x<Map.size()-1;x++){
    for(int y=1;y<Map[x].size()-1;y++){
      if(!Map[x][y].object){
        Map[x][y].dist_from_obj = min( min( min(Map[x-1][y-1].dist_from_obj+sqrt(2.0)*cell_size, Map[x][y-1].dist_from_obj+1.0*cell_size),
                          min(Map[x-1][y+1].dist_from_obj+sqrt(2.0)*cell_size, Map[x-1][y].dist_from_obj+1.0*cell_size)),
                         Map[x][y].dist_from_obj);
      }
    }
  }
  for(int x=Map.size()-2;x>0;x--){
    for(int y=Map[x].size()-2;y>0;y--){
      if(!Map[x][y].object){
        Map[x][y].dist_from_obj = min( min( min(Map[x+1][y].dist_from_obj+1.0*cell_size, Map[x+1][y-1].dist_from_obj+sqrt(2.0)*cell_size),
                          min(Map[x][y+1].dist_from_obj+1.0*cell_size, Map[x+1][y+1].dist_from_obj+sqrt(2.0)*cell_size)),
                         Map[x][y].dist_from_obj);
      }
    }
  }
  
  return true;
}

bool VoronoiPathPlanner::calcDistFromVoronoi(vector<vector<CollisionMapData> >& Map){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }
  
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      if(Map[x][y].voronoi)
        Map[x][y].dist_from_voronoi = 0.0;
      else
        Map[x][y].dist_from_voronoi = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }
  
  for(int x=1;x<Map.size()-1;x++){
    for(int y=1;y<Map[x].size()-1;y++){
      if(!Map[x][y].collision){
        Map[x][y].dist_from_voronoi = min( min( min(Map[x-1][y-1].dist_from_voronoi+sqrt(2.0)*cell_size, Map[x][y-1].dist_from_voronoi+1.0*cell_size),
                          min(Map[x-1][y+1].dist_from_voronoi+sqrt(2.0)*cell_size, Map[x-1][y].dist_from_voronoi+1.0*cell_size)),
                         Map[x][y].dist_from_voronoi);
      }
    }
  }
  for(int x=Map.size()-2;x>0;x--){
    for(int y=Map[x].size()-2;y>0;y--){
      if(!Map[x][y].collision){
        Map[x][y].dist_from_voronoi = min( min( min(Map[x+1][y].dist_from_voronoi+1.0*cell_size, Map[x+1][y-1].dist_from_voronoi+sqrt(2.0)*cell_size),
                          min(Map[x][y+1].dist_from_voronoi+1.0*cell_size, Map[x+1][y+1].dist_from_voronoi+sqrt(2.0)*cell_size)),
                         Map[x][y].dist_from_voronoi);
      }
    }
  }
  
  return true;
}

bool VoronoiPathPlanner::calcDistFromGoal(vector<vector<CollisionMapData> >& Map, vector<double> goal_point){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }
  
  if((goal_point[0]<x_llimit)||(goal_point[0]>x_ulimit)||(goal_point[1]<y_llimit)||(goal_point[1]>y_ulimit)){
    os<<"Error : Goal is out of range"<<endl;
    return false;
  }
  
  int temp_x, temp_y;
  temp_x = (int)round( (goal_point[0] - x_llimit) / cell_size );
  temp_y = (int)round( (goal_point[1] - y_llimit) / cell_size );
  
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      Map[x][y].dist_from_goal = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }
  Map[temp_x][temp_y].dist_from_goal = 0.0;
  
  bool change_flg = true;
  double temp = 0.0;
  
  while(change_flg){
    change_flg = false;
    for(unsigned int i=1;i<Map.size()-1;i++){ //距離値計算 ラスタ走査
      for(unsigned int j=1;j<Map[i].size()-1;j++){
        if(Map[i][j].voronoi){
          temp = Map[i][j].dist_from_goal;
          Map[i][j].dist_from_goal = min( min( min(Map[i-1][j-1].dist_from_goal+sqrt(2.0)*cell_size, Map[i-1][j].dist_from_goal+1.0*cell_size),
                             Map[i-1][j+1].dist_from_goal+sqrt(2.0)*cell_size ),
                          min( Map[i][j-1].dist_from_goal+1.0*cell_size, Map[i][j].dist_from_goal ) );
          if(change_flg==false){
            if(Map[i][j].dist_from_goal!=temp)
              change_flg=true;
          }
        }
      }
    }
    for(int i=Map.size()-2;i>0;i--){ //距離値計算 逆ラスタ走査
      for(int j=Map[i].size()-2;j>0;j--){
        if(Map[i][j].voronoi){
          temp = Map[i][j].dist_from_goal;
          Map[i][j].dist_from_goal = min( min( Map[i][j].dist_from_goal, Map[i][j+1].dist_from_goal+1.0*cell_size ),
                          min( Map[i+1][j-1].dist_from_goal+sqrt(2.0)*cell_size,
                             min( Map[i+1][j].dist_from_goal+1.0*cell_size, Map[i+1][j+1].dist_from_goal+sqrt(2.0)*cell_size ) ) );
          if(change_flg==false){
            if(Map[i][j].dist_from_goal!=temp)
              change_flg=true;
          }
        }
      }
    }
  }
  
  //~ change_flg=true;
    //~ 
  //~ while(change_flg){
    //~ change_flg=false;
    //~ for(unsigned int i=1;i<Map.size()-1;i++){ //距離値計算 ラスタ走査
      //~ for(unsigned int j=1;j<Map[i].size()-1;j++){
        //~ if((!Map[i][j].collision)&&(!Map[i][j].voronoi)){
          //~ temp = Map[i][j].dist_from_goal;
          //~ Map[i][j].dist_from_goal = min( min(Map[i-1][j].dist_from_goal+1.0*cell_size, Map[i][j-1].dist_from_goal+1.0*cell_size),
                          //~ Map[i][j].dist_from_goal );
          //~ if(change_flg==false){
            //~ if(Map[i][j].dist_from_goal!=temp)
              //~ change_flg=true;
          //~ }
        //~ }
      //~ }
    //~ }
    //~ for(int i=Map.size()-2;i>0;i--){ //距離値計算 逆ラスタ走査
      //~ for(int j=Map[i].size()-2;j>0;j--){
        //~ if((!Map[i][j].collision)&&(!Map[i][j].voronoi)){
          //~ temp = Map[i][j].dist_from_goal;
          //~ Map[i][j].dist_from_goal = min( min(Map[i][j+1].dist_from_goal+1.0*cell_size, Map[i+1][j].dist_from_goal+1.0*cell_size),
                          //~ Map[i][j].dist_from_goal );
          //~ if(change_flg==false){
            //~ if(Map[i][j].dist_from_goal!=temp)
              //~ change_flg=true;
          //~ }
        //~ }
      //~ }
    //~ }
  //~ }
  
  return true;
}

bool VoronoiPathPlanner::calcDistFromPath(vector<vector<CollisionMapData> >& Map){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }
  
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      if(Map[x][y].path)
        Map[x][y].dist_from_path = 0.0;
      else
        Map[x][y].dist_from_path = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }
  
  for(int x=1;x<Map.size()-1;x++){
    for(int y=1;y<Map[x].size()-1;y++){
      if(!Map[x][y].collision){
        Map[x][y].dist_from_path = min( min( min(Map[x-1][y-1].dist_from_path+sqrt(2.0)*cell_size, Map[x][y-1].dist_from_path+1.0*cell_size),
                          min(Map[x-1][y+1].dist_from_path+sqrt(2.0)*cell_size, Map[x-1][y].dist_from_path+1.0*cell_size)),
                         Map[x][y].dist_from_path);
      }
    }
  }
  for(int x=Map.size()-2;x>0;x--){
    for(int y=Map[x].size()-2;y>0;y--){
      if(!Map[x][y].collision){
        Map[x][y].dist_from_path = min( min( min(Map[x+1][y].dist_from_path+1.0*cell_size, Map[x+1][y-1].dist_from_path+sqrt(2.0)*cell_size),
                          min(Map[x][y+1].dist_from_path+1.0*cell_size, Map[x+1][y+1].dist_from_path+sqrt(2.0)*cell_size)),
                         Map[x][y].dist_from_path);
      }
    }
  }
  
  return true;
}

bool VoronoiPathPlanner::setCollisionArea(vector<vector<CollisionMapData> >& Map, double threshold){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }
  
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      if(Map[x][y].dist_from_obj<=threshold)
        Map[x][y].collision = true;
      else
        Map[x][y].collision = false;
    }
  }
  
  return true;
}

bool VoronoiPathPlanner::setVoronoiLine(vector<vector<CollisionMapData> >& Map){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }
  
  for(unsigned int x=0;x<Map.size();x++){ 
    for(unsigned int y=0;y<Map[x].size();y++){
      if(!Map[x][y].collision)  Map[x][y].thinning_flg=1;
      else Map[x][y].thinning_flg=0;
    }
  }
  
  bool flg = true;
  while(flg){
    flg=false;
    for(int k=0;k<4;k++){
      for(int x=1;x<Map.size()-1;x++){
        for(int y=1;y<Map[x].size()-1;y++){
          int i, j;
          switch(k){
            case 0:
              i = x+1;
              j = y;
              break;
            case 1:
              i = x;
              j = y-1;
              break;
            case 2:
              i = x-1;
              j = y;
              break;
            case 3:
              i = x;
              j = y+1;
              break;
          }
          if((Map[x][y].thinning_flg==1)&&(Map[i][j].thinning_flg==0)){
            if(((Map[x][y-1].thinning_flg==0)&&(Map[x+1][y].thinning_flg==1)&&(Map[x][y+1].thinning_flg==1)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==1)&&(Map[x-1][y].thinning_flg==1)&&(Map[x+1][y].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==1)&&(Map[x-1][y].thinning_flg==1)&&(Map[x][y+1].thinning_flg==0))||
              ((Map[x-1][y].thinning_flg==0)&&(Map[x+1][y].thinning_flg==1)&&(Map[x][y+1].thinning_flg==1)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y].thinning_flg==0)&&(Map[x+1][y].thinning_flg==0)&&(Map[x][y+1].thinning_flg==1))||
              ((Map[x][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==1)&&(Map[x][y+1].thinning_flg==0))||
              ((Map[x][y-1].thinning_flg==1)&&(Map[x-1][y].thinning_flg==0)&&(Map[x+1][y].thinning_flg==0))||
              ((Map[x][y-1].thinning_flg==0)&&(Map[x+1][y].thinning_flg==1)&&(Map[x][y+1].thinning_flg==0))||
              ((Map[x-1][y].thinning_flg==0)&&(Map[x-1][y+1].thinning_flg==1)&&(Map[x][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==1)&&(Map[x][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==0))||
              ((Map[x][y-1].thinning_flg==0)&&(Map[x+1][y-1].thinning_flg==1)&&(Map[x+1][y].thinning_flg==0))||
              ((Map[x+1][y].thinning_flg==0)&&(Map[x][y+1].thinning_flg==0)&&(Map[x+1][y+1].thinning_flg==1))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==1)&&(Map[x+1][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==1)&&(Map[x+1][y].thinning_flg==1)&&(Map[x-1][y+1].thinning_flg==0)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==1)&&(Map[x+1][y-1].thinning_flg==0)&&(Map[x+1][y].thinning_flg==1)&&(Map[x-1][y+1].thinning_flg==0)&&(Map[x][y+1].thinning_flg==1)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x+1][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==1)&&(Map[x+1][y].thinning_flg==1)&&(Map[x-1][y+1].thinning_flg==0)&&(Map[x][y+1].thinning_flg==1)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==1)&&(Map[x+1][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==1)&&(Map[x-1][y+1].thinning_flg==0)&&(Map[x][y+1].thinning_flg==1)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==0)&&(Map[x+1][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==0)&&(Map[x-1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x][y-1].thinning_flg==0)&&(Map[x+1][y-1].thinning_flg==0)&&(Map[x+1][y].thinning_flg==0)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x-1][y-1].thinning_flg==0)&&(Map[x-1][y].thinning_flg==0)&&(Map[x-1][y+1].thinning_flg==0)&&(Map[x][y+1].thinning_flg==0)&&(Map[x+1][y+1].thinning_flg==0))||
              ((Map[x+1][y-1].thinning_flg==0)&&(Map[x+1][y].thinning_flg==0)&&(Map[x-1][y+1].thinning_flg==0)&&(Map[x][y+1].thinning_flg==0)&&(Map[x+1][y+1].thinning_flg==0))
              )
              Map[x][y].thinning_flg=1;
            else{
              flg=true;
              Map[x][y].thinning_flg=3;
            }
          }
        }
      }
      for(int x=1;x<Map.size()-1;x++){
        for(int y=1;y<Map[x].size()-1;y++){
          if(Map[x][y].thinning_flg==3)
            Map[x][y].thinning_flg=0;
        }
      }
    }
  }
  for(int x=0;x<Map.size();x++){
    for(int y=0;y<Map[x].size();y++){
      if(Map[x][y].thinning_flg==1){
        Map[x][y].voronoi=1;
        Map[x][y].dist_from_voronoi=0.0;
      }
      else Map[x][y].voronoi=0;
    }
  }
  return true;
}

bool VoronoiPathPlanner::connectToVoronoi(vector<vector<CollisionMapData> >& Map, vector<double> connect_point){
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }

  if((connect_point[0]<x_llimit)||(connect_point[0]>x_ulimit)||(connect_point[1]<y_llimit)||(connect_point[1]>y_ulimit)){
    os<<"Error : Connect point is out of range"<<endl;
    return false;
  }
  
  int target_x, target_y, temp_x, temp_y, dx=0, dy=0;
  target_x = (int)round( (connect_point[0] - x_llimit) / cell_size );
  target_y = (int)round( (connect_point[1] - y_llimit) / cell_size );
  temp_x = target_x;
  temp_y = target_y;
  
  double temp_dist = Map[temp_x][temp_y].dist_from_voronoi;
  if(Map[temp_x][temp_y].collision){
    os<<"Error : Connect point is collision"<<endl;
    return false;
  }
  if(temp_dist==0.0)
    return true;
  
  while(temp_dist!=0.0){
    if( Map[temp_x-1][temp_y].dist_from_voronoi < temp_dist ){
      dx = -1;
      dy = 0;
      temp_dist = Map[temp_x+dx][temp_y].dist_from_voronoi;
    }
    if( Map[temp_x][temp_y-1].dist_from_voronoi < temp_dist ){
      dx = 0;
      dy = -1;
      temp_dist = Map[temp_x][temp_y+dy].dist_from_voronoi;
    }
    if( Map[temp_x+1][temp_y].dist_from_voronoi < temp_dist ){
      dx = 1;
      dy = 0;
      temp_dist = Map[temp_x+dx][temp_y].dist_from_voronoi;
    }
    if( Map[temp_x][temp_y+1].dist_from_voronoi < temp_dist ){
      dx = 0;
      dy = 1;
      temp_dist = Map[temp_x][temp_y+dy].dist_from_voronoi;
    }
    if( Map[temp_x-1][temp_y-1].dist_from_voronoi < temp_dist ){
      dx = -1;
      dy = -1;
      temp_dist = Map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }
    if( Map[temp_x-1][temp_y+1].dist_from_voronoi < temp_dist ){
      dx = -1;
      dy = 1;
      temp_dist = Map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }
    if( Map[temp_x+1][temp_y-1].dist_from_voronoi < temp_dist ){
      dx = 1;
      dy = -1;
      temp_dist = Map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }
    if( Map[temp_x+1][temp_y+1].dist_from_voronoi < temp_dist ){
      dx = 1;
      dy = 1;
      temp_dist = Map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }
    temp_x += dx;
    temp_y += dy;
  }
  
  vector<double> vec;
  vec.resize(2);
  vec[0] = temp_x - target_x;
  vec[1] = temp_y - target_y;
  double vec_norm = sqrt( (vec[0]*vec[0])+(vec[1]*vec[1]) );
  vec[0] /= vec_norm;
  vec[1] /= vec_norm;
  
  double temp_norm = 1.0;
  while(temp_norm<vec_norm){
    temp_x = (int)round(target_x + temp_norm*vec[0]);
    temp_y = (int)round(target_y + temp_norm*vec[1]);
    Map[temp_x][temp_y].voronoi = true;
    temp_norm += 1.0;
  }
  temp_x = (int)round(target_x + vec_norm*vec[0]);
  temp_y = (int)round(target_y + vec_norm*vec[1]);
  Map[temp_x][temp_y].voronoi = true;
  
  Map[target_x][target_y].voronoi = true;
  
  return true;
}

bool VoronoiPathPlanner::calcVoronoiPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path){
  out_path.clear();
  
  if(Map.empty()){
    os<<"Error : Map is empty"<<endl;
    return false;
  }

  if((start[0]<x_llimit)||(start[0]>x_ulimit)||(start[1]<y_llimit)||(start[1]>y_ulimit)){
    os<<"Error : Start point is out of range"<<endl;
    return false;
  }
  if((goal[0]<x_llimit)||(goal[0]>x_ulimit)||(goal[1]<y_llimit)||(goal[1]>y_ulimit)){
    os<<"Error : Goal point is out of range"<<endl;
    return false;
  }
  
  int i_start_x, i_start_y, i_goal_x, i_goal_y, i_temp_x, i_temp_y, dx=0, dy=0;
  i_start_x = (int)round( (start[0] - x_llimit) / cell_size );
  i_start_y = (int)round( (start[1] - y_llimit) / cell_size );
  i_goal_x = (int)round( (goal[0] - x_llimit) / cell_size );
  i_goal_y = (int)round( (goal[1] - y_llimit) / cell_size );
  
  if(Map[i_start_x][i_start_y].collision){
    os<<"Error : Start is collision"<<endl;
    return false;
  }
  if(Map[i_goal_x][i_goal_y].collision){
    os<<"Error : Goal is collision"<<endl;
    return false;
  }
  Map[i_start_x][i_start_y].path = true;
  Map[i_goal_x][i_goal_y].path = true;
  
  //~ out_path.push_back(start);
  
  i_temp_x = i_start_x;
  i_temp_y = i_start_y;
  double temp_dist = Map[i_temp_x][i_temp_y].dist_from_goal;
  vector<double> temp_pos;
  temp_pos.resize(2);
  
  if(temp_dist==0)
    return true;
  
  while(1){
    if(temp_dist > Map[i_temp_x-1][i_temp_y].dist_from_goal){
      dx = -1;
      dy = 0;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x][i_temp_y-1].dist_from_goal){
      dx = 0;
      dy = -1;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x+1][i_temp_y].dist_from_goal){
      dx = 1;
      dy = 0;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x][i_temp_y+1].dist_from_goal){
      dx = 0;
      dy = 1;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x-1][i_temp_y-1].dist_from_goal){
      dx = -1;
      dy = -1;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x-1][i_temp_y+1].dist_from_goal){
      dx = -1;
      dy = 1;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x+1][i_temp_y-1].dist_from_goal){
      dx = 1;
      dy = -1;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    if(temp_dist > Map[i_temp_x+1][i_temp_y+1].dist_from_goal){
      dx = 1;
      dy = 1;
      temp_dist = Map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }
    i_temp_x += dx;
    i_temp_y += dy;
    Map[i_temp_x][i_temp_y].path = true;
    
    if(temp_dist==0.0)
      break;
    
    temp_pos[0] = (i_temp_x * cell_size) + x_llimit;
    temp_pos[1] = (i_temp_y * cell_size) + y_llimit;
    out_path.push_back(temp_pos);
  }
  
  //~ out_path.push_back(goal);
  
  //~ for(int i=0;i<out_path.size();i++){
    //~ cout<<out_path[i][0]<<"  "<<out_path[i][1]<<endl;
  //~ }
  
  return true;
}

bool VoronoiPathPlanner::smoothVoronoiPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> > in_path, vector<vector<double> >& out_path, double threshold){
  out_path.clear();
  vector<double> temp_pos;
  temp_pos.resize(3);
  
  if(in_path.size()<2){
    out_path.push_back(start);
    out_path.push_back(goal);
    return true;
  }
  
  out_path.push_back(start);
  temp_pos = start;
  
  int i_temp_x, i_temp_y, i_prev_x, i_prev_y, i_next_x, i_next_y;
  double temp_th = 0.0;
  i_prev_x = (int)round( (start[0] - x_llimit) / cell_size );
  i_prev_y = (int)round( (start[1] - y_llimit) / cell_size );
  
  vector<double> vec;
  vec.resize(2);
  double vec_norm, temp_norm;
  bool smoothing = true;
  
  for(unsigned int i=1;i<in_path.size();i++){
    smoothing = true;
    i_next_x = (int)round( (in_path[i][0] - x_llimit) / cell_size );
    i_next_y = (int)round( (in_path[i][1] - y_llimit) / cell_size );
    
    vec[0] = i_next_x - i_prev_x;
    vec[1] = i_next_y - i_prev_y;
    vec_norm = sqrt( (vec[0]*vec[0])+(vec[1]*vec[1]) );
    vec[0] /= vec_norm;
    vec[1] /= vec_norm;
    
    temp_norm = 1.0;
    while(temp_norm<vec_norm){
      i_temp_x = (int)round(i_prev_x + temp_norm*vec[0]);
      i_temp_y = (int)round(i_prev_y + temp_norm*vec[1]);
      if(Map[i_temp_x][i_temp_y].dist_from_voronoi>threshold){
        smoothing = false;
        break;
      }
      temp_norm += 1.0;
    }
    
    if(!smoothing){
      temp_pos[2] = atan2(i_next_y - i_prev_y, i_next_x - i_prev_x);
      out_path.push_back(temp_pos);
      temp_pos[0] = (i_next_x * cell_size) + x_llimit;
      temp_pos[1] = (i_next_y * cell_size) + y_llimit;
      out_path.push_back(temp_pos);
      i_prev_x = i_next_x;
      i_prev_y = i_next_y;
    }
  }
  temp_pos[2] = atan2(goal[1] - ((i_prev_y * cell_size) + y_llimit), goal[0] - ((i_prev_x * cell_size) + x_llimit));
  out_path.push_back(temp_pos);
  temp_pos[0] = goal[0];
  temp_pos[1] = goal[1];
  out_path.push_back(temp_pos);

  out_path.push_back(goal);
  //~ for(int i=0;i<out_path.size();i++){
    //~ cout<<out_path[i][0]<<"  "<<out_path[i][1]<<"  "<<rad2deg(out_path[i][2])<<endl;
  //~ }
  
  return true;
}

bool VoronoiPathPlanner::compVoronoiPath(vector<vector<double> > in_path, vector<vector<double> >& out_path){
  out_path.clear();
  
  if(in_path.size()<2){
    os<<"Error : path is empty"<<endl;
    return false;
  }
  
  double incre_th = deg2rad(1.0);
  vector<double> vec, temp_pos, next_pos;
  vec.resize(3);
  temp_pos.resize(3);
  next_pos.resize(3);
  double vec_norm, temp_norm;
  
  for(unsigned int i=0;i<in_path.size()-1;i++){
    temp_pos[0]=in_path[i][0];
    temp_pos[1]=in_path[i][1];
    temp_pos[2]=in_path[i][2];
    out_path.push_back(temp_pos);
    
    next_pos[0]=in_path[i+1][0];
    next_pos[1]=in_path[i+1][1];
    next_pos[2]=in_path[i+1][2];
    
    vec[0] = next_pos[0] - temp_pos[0];
    vec[1] = next_pos[1] - temp_pos[1];
    vec[2] = next_pos[2] - temp_pos[2];
    vec_norm = sqrt( (vec[0]*vec[0])+(vec[1]*vec[1]) );
    if(vec_norm!=0.0){
      vec[0] /= vec_norm;
      vec[1] /= vec_norm;
    }
    if(vec[2]>M_PI){
      vec[2] -= 2*M_PI;
      next_pos[2] -= 2*M_PI;
    } 
    if(vec[2]<-M_PI){
      vec[2] += 2*M_PI;
      next_pos[2] += 2*M_PI;
    }
    
    while(1){
      if(vec[2]>=0){
        temp_pos[2] += incre_th;
        if(temp_pos[2]>next_pos[2])  break;
      }
      else{
        temp_pos[2] -= incre_th;
        if(temp_pos[2]<next_pos[2])  break;
      }
      out_path.push_back(temp_pos);
    }
    temp_pos[2]=next_pos[2];
    out_path.push_back(temp_pos);
    
    temp_norm = 0.01;
    while(temp_norm<vec_norm){
      temp_pos[0] = in_path[i][0] + temp_norm*vec[0];
      temp_pos[1] = in_path[i][1] + temp_norm*vec[1];
      temp_norm += 0.01;
      out_path.push_back(temp_pos);
    }
    out_path.push_back(next_pos);
  }
  
  temp_pos[0]=next_pos[0];
  temp_pos[1]=next_pos[1];
  temp_pos[2]=next_pos[2];
  
  next_pos[0]=in_path[in_path.size()-1][0];
  next_pos[1]=in_path[in_path.size()-1][1];
  next_pos[2]=in_path[in_path.size()-1][2];
  
  vec[0] = next_pos[0] - temp_pos[0];
  vec[1] = next_pos[1] - temp_pos[1];
  vec[2] = next_pos[2] - temp_pos[2];
  vec_norm = sqrt( (vec[0]*vec[0])+(vec[1]*vec[1]) );
  if(vec_norm!=0.0){
    vec[0] /= vec_norm;
    vec[1] /= vec_norm;
  }
  if(vec[2]>M_PI){
    vec[2] -= 2*M_PI;
    next_pos[2] -= 2*M_PI;
  } 
  if(vec[2]<-M_PI){
    vec[2] += 2*M_PI;
    next_pos[2] += 2*M_PI;
  }
  
  while(1){
    if(vec[2]>=0){
      temp_pos[2] += incre_th;
      if(temp_pos[2]>next_pos[2])  break;
    }
    else{
      temp_pos[2] -= incre_th;
      if(temp_pos[2]<next_pos[2])  break;
    }
    out_path.push_back(temp_pos);
  }
  temp_pos[2]=next_pos[2];
  out_path.push_back(temp_pos);
  
  //~ for(int i=0;i<out_path.size();i++){
    //~ cout<<out_path[i][0]<<"  "<<out_path[i][1]<<"  "<<rad2deg(out_path[i][2])<<endl;
  //~ }
  
  return true;
}

bool VoronoiPathPlanner::planVoronoiPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path){
  initCollisionMap(Map);
  calcDistFromObj(Map);
  setCollisionArea(Map, 0.4);
  setVoronoiLine(Map);
  calcDistFromVoronoi(Map);
  connectToVoronoi(Map, start);
  calcDistFromVoronoi(Map);
  connectToVoronoi(Map, goal);
  calcDistFromVoronoi(Map);
  calcDistFromGoal(Map, goal);
  calcVoronoiPath(Map, start, goal, out_path);
  vector<vector<double> > smooth_path, comp_path;
  smoothVoronoiPath(Map, start, goal, out_path, smooth_path, 0.1);
  compVoronoiPath(smooth_path, comp_path);
  
  MapOutput(Map);
  
  out_path.clear();
  out_path = comp_path;
  return true;
}

bool VoronoiPathPlanner::initCollisionMap(vector<vector<CollisionMapData> >& Map){
  //QString useMapName = QDir::currentPath() + "/extplugin/rostmsPlugin/libVoronoiPathPlanner/CollisionMap/use_collision_map.csv";
  QString useMapName = "/home/rt/catkin_ws/src/ros_tms/tms_rp/tms_rp_rostms_plugin/src/libVoronoiPathPlanner/CollisionMap/use_collision_map.csv";
  QFile use_file(useMapName);
  if (!use_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    cout << "can not open file." << endl;
    return false;
  }
  
  Map.clear();
  vector<CollisionMapData> tempMapLine;
  CollisionMapData tempMapData;
  
  QTextStream in( &use_file );
  QString line;
  char *tp, templine[4096];
  line = in.readLine();
  sprintf(templine, line.toStdString().c_str());
  
  tp = strtok(templine, ",");
  x_llimit = atof(tp);
  tp = strtok(NULL, ",");
  x_ulimit = atof(tp);
  tp = strtok(NULL, ",");
  y_llimit = atof(tp);
  tp = strtok(NULL, ",");
  y_ulimit = atof(tp);
  tp = strtok(NULL, ",");
  cell_size = atof(tp);
  
  while (!in.atEnd()) {
    tempMapLine.clear();
    line = in.readLine();
    sprintf(templine, line.toStdString().c_str());
    tp = strtok(templine, ",");
    tempMapData.object = atoi(tp);
    if(tempMapData.object==1){
      tempMapData.dist_from_obj = 0.0;
      tempMapData.collision = true;
    }
    else{
      tempMapData.dist_from_obj = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
      tempMapData.collision = false;
    }
    tempMapData.voronoi = false;
    tempMapData.path = false;
    tempMapData.thinning_flg = 0;
    tempMapData.dist_from_voronoi = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    tempMapData.dist_from_goal = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    tempMapData.dist_from_path = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    tempMapLine.push_back(tempMapData);
    
    tp = strtok(NULL, ",");
    while(tp != NULL){
      tempMapData.object = atoi(tp);
      if(tempMapData.object==1){
        tempMapData.dist_from_obj = 0.0;
        tempMapData.collision = true;
      }
      else{
        tempMapData.dist_from_obj = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
        tempMapData.collision = false;
      }
      tempMapData.voronoi = false;
      tempMapData.path = false;
      tempMapData.thinning_flg = 0;
      tempMapData.dist_from_voronoi = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
      tempMapData.dist_from_goal = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
      tempMapData.dist_from_path = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
      tempMapLine.push_back(tempMapData);
      tp = strtok(NULL, ",");
    }
    Map.push_back(tempMapLine);
  }
  
  use_file.close();
  
  //~ for(unsigned int x=0;x<Map.size();x++){
    //~ for(unsigned int y=0;y<Map[x].size();y++){
      //~ cout<<Map[x][y].object;
    //~ }
    //~ cout<<endl;
  //~ }
  
  return true;
}

//~ void VoronoiPathPlanner::setCollisionMap(vector<vector<CollisionMapData> >& Map){
  //~ 
//~ }

bool VoronoiPathPlanner::loadCollisionMap(QString select_file_name){
  //QString useMapName = QDir::currentPath() + "/extplugin/rostmsPlugin/libVoronoiPathPlanner/CollisionMap/use_collision_map.csv";
  QString useMapName = "/home/rt/catkin_ws/src/ros_tms/tms_rp/tms_rp_rostms_plugin/src/libVoronoiPathPlanner/CollisionMap/use_collision_map.csv";
  QFile select_file(select_file_name);
  QFile use_file(useMapName);
  if ((!select_file.open(QIODevice::ReadOnly | QIODevice::Text))||(!use_file.open(QIODevice::WriteOnly | QIODevice::Text))) {
    cout << "can not open file." << endl;
    return false;
  }
  QTextStream in( &select_file );
  while (!in.atEnd()) {
    QString line = in.readLine();
    //~ cout << line.toStdString() << endl;
    use_file.write(line.toStdString().c_str());
    use_file.write("\n");
  }
  select_file.close();
  use_file.close();
  
  return true;
}

bool VoronoiPathPlanner::makeCollisionMap(vector<vector<int> >& out_collision_map){
  if(cell_size<=0.0){
    os<<"Error : please set cell_size more than 0"<<endl;
    return false;
  }
  out_collision_map.clear();
  initialCollision();
  
  vector<int>  temp_line;
  temp_line.clear();
  
  for(double x = x_llimit;x<=x_ulimit+1.0e-10;x=x+cell_size){
    temp_line.clear();
    for(double y = y_llimit;y<=y_ulimit+1.0e-10;y=y+cell_size){
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

