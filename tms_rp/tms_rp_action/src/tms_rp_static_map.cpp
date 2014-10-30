#include <tms_rp_static_map.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
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
                                  os(MessageView::mainInstance()->cout()),
                                  tac(*TmsRpController::instance()) {
  sid = 100000;

  static ros::NodeHandle nh;

  pp_map_pub = nh.advertise<tms_msg_rp::rps_map_full>("rps_map_data", 1);

  InitCollisionMap(map);
  SetVoronoiLine(map, msg);
  CalcDistFromObj(map, msg);
  ConvertMap(map, pub_map);
}

//------------------------------------------------------------------------------
TmsRpStaticMap::~TmsRpStaticMap()
{
}

//------------------------------------------------------------------------------
bool TmsRpStaticMap::InitCollisionMap(vector<vector<CollisionMapData> >& map){
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
  x_llimit = atof(tp);
  tp = strtok(NULL, ",");
  x_ulimit = atof(tp);
  tp = strtok(NULL, ",");
  y_llimit = atof(tp);
  tp = strtok(NULL, ",");
  y_ulimit = atof(tp);
  tp = strtok(NULL, ",");
  cell_size = atof(tp);

  int CommaCount=0;

  while(fgets(buff, 4096, fp) != NULL){ //collision
    CommaCount = 0;
    tp = strtok(buff, ",");
    tempMapData.object=atof(tp);

    if(tempMapData.object){
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

    CommaCount++;

    while(tp != NULL){

      if(CommaCount == int(round((y_ulimit-y_llimit)/cell_size))+1)
        break;

      tp = strtok(NULL, ",");
      CommaCount++;

      if(tp != NULL){
        tempMapData.object=atof(tp);
        if(tempMapData.object){
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
      }
    }

    map.push_back(tempMapLine);
    tempMapLine.clear();
  }

  fclose(fp);

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpStaticMap::SetVoronoiLine(vector<vector<CollisionMapData> >& map, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(unsigned int x=0;x<map.size();x++){
    for(unsigned int y=0;y<map[x].size();y++){
      map[x][y].voronoi = false;
      if(!map[x][y].collision)
        map[x][y].thinning_flg=1;
      else
        map[x][y].thinning_flg=0;
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

          if((map[x][y].thinning_flg==1)&&(map[i][j].thinning_flg==0)){
            if(((map[x][y-1].thinning_flg==0)&&(map[x+1][y].thinning_flg==1)&&(map[x][y+1].thinning_flg==1)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==1)&&(map[x-1][y].thinning_flg==1)&&(map[x+1][y].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==1)&&(map[x-1][y].thinning_flg==1)&&(map[x][y+1].thinning_flg==0))||
            ((map[x-1][y].thinning_flg==0)&&(map[x+1][y].thinning_flg==1)&&(map[x][y+1].thinning_flg==1)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y].thinning_flg==0)&&(map[x+1][y].thinning_flg==0)&&(map[x][y+1].thinning_flg==1))||
            ((map[x][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==1)&&(map[x][y+1].thinning_flg==0))||
            ((map[x][y-1].thinning_flg==1)&&(map[x-1][y].thinning_flg==0)&&(map[x+1][y].thinning_flg==0))||
            ((map[x][y-1].thinning_flg==0)&&(map[x+1][y].thinning_flg==1)&&(map[x][y+1].thinning_flg==0))||
            ((map[x-1][y].thinning_flg==0)&&(map[x-1][y+1].thinning_flg==1)&&(map[x][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==1)&&(map[x][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==0))||
            ((map[x][y-1].thinning_flg==0)&&(map[x+1][y-1].thinning_flg==1)&&(map[x+1][y].thinning_flg==0))||
            ((map[x+1][y].thinning_flg==0)&&(map[x][y+1].thinning_flg==0)&&(map[x+1][y+1].thinning_flg==1))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==1)&&(map[x+1][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==1)&&(map[x+1][y].thinning_flg==1)&&(map[x-1][y+1].thinning_flg==0)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==1)&&(map[x+1][y-1].thinning_flg==0)&&(map[x+1][y].thinning_flg==1)&&(map[x-1][y+1].thinning_flg==0)&&(map[x][y+1].thinning_flg==1)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x+1][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==1)&&(map[x+1][y].thinning_flg==1)&&(map[x-1][y+1].thinning_flg==0)&&(map[x][y+1].thinning_flg==1)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==1)&&(map[x+1][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==1)&&(map[x-1][y+1].thinning_flg==0)&&(map[x][y+1].thinning_flg==1)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==0)&&(map[x+1][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==0)&&(map[x-1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x][y-1].thinning_flg==0)&&(map[x+1][y-1].thinning_flg==0)&&(map[x+1][y].thinning_flg==0)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x-1][y-1].thinning_flg==0)&&(map[x-1][y].thinning_flg==0)&&(map[x-1][y+1].thinning_flg==0)&&(map[x][y+1].thinning_flg==0)&&(map[x+1][y+1].thinning_flg==0))||
            ((map[x+1][y-1].thinning_flg==0)&&(map[x+1][y].thinning_flg==0)&&(map[x-1][y+1].thinning_flg==0)&&(map[x][y+1].thinning_flg==0)&&(map[x+1][y+1].thinning_flg==0))
            )
              map[x][y].thinning_flg=1;
            else {
              flg=true;
              map[x][y].thinning_flg=3;
            }
          }
        }
      }
      for(int x=1;x<map.size()-1;x++){
        for(int y=1;y<map[x].size()-1;y++){
          if(map[x][y].thinning_flg==3)
          map[x][y].thinning_flg=0;
        }
      }
    }
  }

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      if(map[x][y].thinning_flg==1){
        map[x][y].voronoi=1;
        map[x][y].dist_from_voronoi=0.0;
      }
      else
        map[x][y].voronoi=0;
    }
  }
  return true;
}

//------------------------------------------------------------------------------
bool TmsRpStaticMap::CalcDistFromObj(vector<vector<CollisionMapData> >& map, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      if(map[x][y].object)
        map[x][y].dist_from_obj = 0.0;
      else
        map[x][y].dist_from_obj = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }

  for(int x=1;x<map.size()-1;x++){
    for(int y=1;y<map[x].size()-1;y++){
      if(!map[x][y].object){
        map[x][y].dist_from_obj = min( min( min(map[x-1][y-1].dist_from_obj+sqrt(2.0)*cell_size, map[x][y-1].dist_from_obj+1.0*cell_size),
        min(map[x-1][y+1].dist_from_obj+sqrt(2.0)*cell_size, map[x-1][y].dist_from_obj+1.0*cell_size)),
        map[x][y].dist_from_obj);
      }
    }
  }

  for(int x=map.size()-2;x>0;x--){
    for(int y=map[x].size()-2;y>0;y--){
      if(!map[x][y].object){
        map[x][y].dist_from_obj = min( min( min(map[x+1][y].dist_from_obj+1.0*cell_size, map[x+1][y-1].dist_from_obj+sqrt(2.0)*cell_size),
        min(map[x][y+1].dist_from_obj+1.0*cell_size, map[x+1][y+1].dist_from_obj+sqrt(2.0)*cell_size)),
        map[x][y].dist_from_obj);
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void TmsRpStaticMap::ConvertMap(vector<vector<CollisionMapData> > map, tms_msg_rp::rps_map_full& pp_map){
  pp_map.rps_map_x.clear();

  pp_map.x_llimit = x_llimit;
  pp_map.x_ulimit = x_ulimit;
  pp_map.y_llimit = y_llimit;
  pp_map.y_ulimit = y_ulimit;
  pp_map.cell_size = cell_size;

  tms_msg_rp::rps_map_data temp_map_d;
  tms_msg_rp::rps_map_y temp_map_y;

  for(unsigned int x=0;x<map.size();x++){
    temp_map_y.rps_map_y.clear();
    for(unsigned int y=0;y<map[x].size();y++){
      temp_map_d.object = map[x][y].object;
      temp_map_d.voronoi = map[x][y].voronoi;
      temp_map_d.dist_from_obj_f = map[x][y].dist_from_obj;

      temp_map_y.rps_map_y.push_back(temp_map_d);
    }
    pp_map.rps_map_x.push_back(temp_map_y);
  }
}

//------------------------------------------------------------------------------
void TmsRpStaticMap::MapPublish(){
  pp_map_pub.publish(pub_map);
}

//------------------------------------------------------------------------------
