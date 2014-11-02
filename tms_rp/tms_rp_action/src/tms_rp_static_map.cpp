#include <tms_rp_static_map.h>

//------------------------------------------------------------------------------
using namespace grasp;
using namespace tms_rp;

//------------------------------------------------------------------------------
TmsRpStaticMap* TmsRpStaticMap::instance()
{
  static TmsRpStaticMap* instance = new TmsRpStaticMap();
  return instance;
}

//------------------------------------------------------------------------------
TmsRpStaticMap::TmsRpStaticMap(): ToolBar("TmsRpStaticMap"),
                                  os_(MessageView::mainInstance()->cout()),
                                  tac_(*TmsRpController::instance()) {
  sid_ = 100000;

  static ros::NodeHandle nh;

  pp_map_pub_ = nh.advertise<tms_msg_rp::rps_map_full>("rps_map_data", 1);

  initCollisionMap(collision_map_);
  setVoronoiLine(collision_map_, result_msg_);
  calcDistFromObj(collision_map_, result_msg_);
  convertMap(collision_map_, pub_map_);
}

//------------------------------------------------------------------------------
TmsRpStaticMap::~TmsRpStaticMap()
{
}

//------------------------------------------------------------------------------
bool TmsRpStaticMap::initCollisionMap(vector<vector<CollisionMapData> >& map){
  FILE *fp;
  string file_name;
  char home_dir[255];
  const char *fname;

  strcpy(home_dir, getenv("HOME"));
  file_name = home_dir;
  file_name += "/catkin_ws/src/ros_tms/tms_rp/tms_rp_action/map/use_collision_map.csv";
  fname = file_name.c_str();

  while(1){
    if((fp = fopen(fname, "r")) == NULL){
      cout<<"Error: collision map cannot open"<<endl;
      return false;
    }
    else
      break;
  }

  map.clear();
  vector<CollisionMapData> tempMapLine;
  CollisionMapData tempMapData;

  char *tp, buff[4096];

  if(fgets(buff, 4096, fp) == NULL)
    return false;

  tp = strtok(buff, ",");
  x_llimit_ = atof(tp);
  tp = strtok(NULL, ",");
  x_ulimit_ = atof(tp);
  tp = strtok(NULL, ",");
  y_llimit_ = atof(tp);
  tp = strtok(NULL, ",");
  y_ulimit_ = atof(tp);
  tp = strtok(NULL, ",");
  cell_size_ = atof(tp);

  int CommaCount=0;

  while(fgets(buff, 4096, fp) != NULL){ //collision
    CommaCount = 0;
    tp = strtok(buff, ",");
    tempMapData.object_=atof(tp);

    if(tempMapData.object_){
      tempMapData.dist_from_obj_ = 0.0;
      tempMapData.collision_ = true;
    }
    else{
      tempMapData.dist_from_obj_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
      tempMapData.collision_ = false;
    }

    tempMapData.voronoi_ = false;
    tempMapData.path_ = false;
    tempMapData.thinning_flg_ = 0;
    tempMapData.dist_from_voronoi_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
    tempMapData.dist_from_goal_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
    tempMapData.dist_from_path_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
    tempMapLine.push_back(tempMapData);

    CommaCount++;

    while(tp != NULL){

      if(CommaCount == int(round((y_ulimit_-y_llimit_)/cell_size_))+1)
        break;

      tp = strtok(NULL, ",");
      CommaCount++;

      if(tp != NULL){
        tempMapData.object_=atof(tp);
        if(tempMapData.object_){
          tempMapData.dist_from_obj_ = 0.0;
          tempMapData.collision_ = true;
        }
        else{
          tempMapData.dist_from_obj_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
          tempMapData.collision_ = false;
        }

        tempMapData.voronoi_ = false;
        tempMapData.path_ = false;
        tempMapData.thinning_flg_ = 0;
        tempMapData.dist_from_voronoi_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
        tempMapData.dist_from_goal_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
        tempMapData.dist_from_path_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
        tempMapLine.push_back(tempMapData);
      }
    }

    map.push_back(tempMapLine);
    tempMapLine.clear();
  }

  fclose(fp);

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpStaticMap::setVoronoiLine(vector<vector<CollisionMapData> >& map, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(unsigned int x=0;x<map.size();x++){
    for(unsigned int y=0;y<map[x].size();y++){
      map[x][y].voronoi_ = false;
      if(!map[x][y].collision_)
        map[x][y].thinning_flg_=1;
      else
        map[x][y].thinning_flg_=0;
    }
  }

  bool flg = true;

  while(flg){
    flg=false;
    for(int k=0;k<4;k++){
      for(int x=1;x<map.size()-1;x++){
        for(int y=1;y<map[x].size()-1;y++){
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

          if((map[x][y].thinning_flg_==1)&&(map[i][j].thinning_flg_==0)){
            if(((map[x][y-1].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==1)&&(map[x][y+1].thinning_flg_==1)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==1)&&(map[x-1][y].thinning_flg_==1)&&(map[x+1][y].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==1)&&(map[x-1][y].thinning_flg_==1)&&(map[x][y+1].thinning_flg_==0))||
            ((map[x-1][y].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==1)&&(map[x][y+1].thinning_flg_==1)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==1))||
            ((map[x][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==1)&&(map[x][y+1].thinning_flg_==0))||
            ((map[x][y-1].thinning_flg_==1)&&(map[x-1][y].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==0))||
            ((map[x][y-1].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==1)&&(map[x][y+1].thinning_flg_==0))||
            ((map[x-1][y].thinning_flg_==0)&&(map[x-1][y+1].thinning_flg_==1)&&(map[x][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==1)&&(map[x][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==0))||
            ((map[x][y-1].thinning_flg_==0)&&(map[x+1][y-1].thinning_flg_==1)&&(map[x+1][y].thinning_flg_==0))||
            ((map[x+1][y].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==0)&&(map[x+1][y+1].thinning_flg_==1))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==1)&&(map[x+1][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==1)&&(map[x+1][y].thinning_flg_==1)&&(map[x-1][y+1].thinning_flg_==0)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==1)&&(map[x+1][y-1].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==1)&&(map[x-1][y+1].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==1)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x+1][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==1)&&(map[x+1][y].thinning_flg_==1)&&(map[x-1][y+1].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==1)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==1)&&(map[x+1][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==1)&&(map[x-1][y+1].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==1)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==0)&&(map[x+1][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==0)&&(map[x-1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x][y-1].thinning_flg_==0)&&(map[x+1][y-1].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==0)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x-1][y-1].thinning_flg_==0)&&(map[x-1][y].thinning_flg_==0)&&(map[x-1][y+1].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==0)&&(map[x+1][y+1].thinning_flg_==0))||
            ((map[x+1][y-1].thinning_flg_==0)&&(map[x+1][y].thinning_flg_==0)&&(map[x-1][y+1].thinning_flg_==0)&&(map[x][y+1].thinning_flg_==0)&&(map[x+1][y+1].thinning_flg_==0))
            )
              map[x][y].thinning_flg_=1;
            else {
              flg=true;
              map[x][y].thinning_flg_=3;
            }
          }
        }
      }
      for(int x=1;x<map.size()-1;x++){
        for(int y=1;y<map[x].size()-1;y++){
          if(map[x][y].thinning_flg_==3)
          map[x][y].thinning_flg_=0;
        }
      }
    }
  }

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      if(map[x][y].thinning_flg_==1){
        map[x][y].voronoi_=1;
        map[x][y].dist_from_voronoi_=0.0;
      }
      else
        map[x][y].voronoi_=0;
    }
  }
  return true;
}

//------------------------------------------------------------------------------
bool TmsRpStaticMap::calcDistFromObj(vector<vector<CollisionMapData> >& map, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      if(map[x][y].object_)
        map[x][y].dist_from_obj_ = 0.0;
      else
        map[x][y].dist_from_obj_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
    }
  }

  for(int x=1;x<map.size()-1;x++){
    for(int y=1;y<map[x].size()-1;y++){
      if(!map[x][y].object_){
        map[x][y].dist_from_obj_ = min( min( min(map[x-1][y-1].dist_from_obj_+sqrt(2.0)*cell_size_, map[x][y-1].dist_from_obj_+1.0*cell_size_),
        min(map[x-1][y+1].dist_from_obj_+sqrt(2.0)*cell_size_, map[x-1][y].dist_from_obj_+1.0*cell_size_)),
        map[x][y].dist_from_obj_);
      }
    }
  }

  for(int x=map.size()-2;x>0;x--){
    for(int y=map[x].size()-2;y>0;y--){
      if(!map[x][y].object_){
        map[x][y].dist_from_obj_ = min( min( min(map[x+1][y].dist_from_obj_+1.0*cell_size_, map[x+1][y-1].dist_from_obj_+sqrt(2.0)*cell_size_),
        min(map[x][y+1].dist_from_obj_+1.0*cell_size_, map[x+1][y+1].dist_from_obj_+sqrt(2.0)*cell_size_)),
        map[x][y].dist_from_obj_);
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void TmsRpStaticMap::convertMap(vector<vector<CollisionMapData> > map, tms_msg_rp::rps_map_full& pp_map){
  pp_map.rps_map_x.clear();

  pp_map.x_llimit = x_llimit_;
  pp_map.x_ulimit = x_ulimit_;
  pp_map.y_llimit = y_llimit_;
  pp_map.y_ulimit = y_ulimit_;
  pp_map.cell_size = cell_size_;

  tms_msg_rp::rps_map_data temp_map_d;
  tms_msg_rp::rps_map_y temp_map_y;

  for(unsigned int x=0;x<map.size();x++){
    temp_map_y.rps_map_y.clear();
    for(unsigned int y=0;y<map[x].size();y++){
      temp_map_d.object = map[x][y].object_;
      temp_map_d.voronoi = map[x][y].voronoi_;
      temp_map_d.dist_from_obj_f = map[x][y].dist_from_obj_;

      temp_map_y.rps_map_y.push_back(temp_map_d);
    }
    pp_map.rps_map_x.push_back(temp_map_y);
  }
}

//------------------------------------------------------------------------------
void TmsRpStaticMap::mapPublish(){
  pp_map_pub_.publish(pub_map_);
}

//------------------------------------------------------------------------------
