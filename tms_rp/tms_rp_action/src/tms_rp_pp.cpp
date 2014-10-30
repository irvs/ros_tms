#include <tms_rp_pp.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

//------------------------------------------------------------------------------
TmsRpPathPlanning* TmsRpPathPlanning::instance()
{
  static TmsRpPathPlanning* instance = new TmsRpPathPlanning();
  return instance;
}

//------------------------------------------------------------------------------
TmsRpPathPlanning::TmsRpPathPlanning(): kSmartPal5CollisionThreshold_(400.0),
                                        kSmartPal4CollisionThreshold_(400.0),
                                        kKobukiCollisionThreshold_(200.0),
                                        kKKPCollisionThreshold_(400.0),
                                        kRobotControlWagonDist_(400.0),
                                        kSmoothVoronoiPathThreshold_(100.0),
                                        kPushWagonPathThreshold_(200.0),
                                        ToolBar("TmsRpPathPlanning"),
                                        os_(MessageView::mainInstance()->cout()),
                                        tac_(*TmsRpController::instance()) {
  static ros::NodeHandle nh;

  static_map_sub_ = nh.subscribe("/rps_map_data", 10, &TmsRpPathPlanning::GetStaticMap, this);
  service_voronoi_path_ = nh.advertiseService("/rps_voronoi_path_planning", &TmsRpPathPlanning::VoronoiPathPlanner, this);
  robot_path_pub_ = nh.advertise<tms_msg_rp::rps_route>("/rps_robot_path", 1);

}

//------------------------------------------------------------------------------
TmsRpPathPlanning::~TmsRpPathPlanning()
{
}

//------------------------------------------------------------------------------
void TmsRpPathPlanning::GetStaticMap(const tms_msg_rp::rps_map_full::ConstPtr& original_map){
  static_map_.clear();

  x_llimit  = original_map->x_llimit;
  x_ulimit  = original_map->x_ulimit;
  y_llimit  = original_map->y_llimit;
  y_ulimit  = original_map->y_ulimit;
  cell_size = original_map->cell_size;

  vector<CollisionMapData> temp_map_line;
  CollisionMapData temp_map_d;

  for(unsigned int x=0;x<original_map->rps_map_x.size();x++){
    temp_map_line.clear();
    for(unsigned int y=0;y<original_map->rps_map_x[x].rps_map_y.size();y++){
      temp_map_d.object         = original_map->rps_map_x[x].rps_map_y[y].object;
      temp_map_d.collision      = original_map->rps_map_x[x].rps_map_y[y].object;
      temp_map_d.voronoi        = original_map->rps_map_x[x].rps_map_y[y].voronoi;
      temp_map_d.dist_from_obj	= original_map->rps_map_x[x].rps_map_y[y].dist_from_obj_f;

      temp_map_line.push_back(temp_map_d);
    }
    static_map_.push_back(temp_map_line);
  }
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::VoronoiPathPlanner(tms_msg_rp::rps_voronoi_path_planning::Request& req,
                                           tms_msg_rp::rps_voronoi_path_planning::Response& res)
{
  res.success = 0;
  res.VoronoiPath.clear();
  res.voronoi_path.rps_route.clear();
  ROS_INFO("Start Voronoi Path Plan...");
  //~ clock_t t_start,t_end;
  //~ t_start = clock();

  double collision_threshold = GetRobotCollisionThreshold(req.robot_id);

  vector<double> start, goal;
  start.resize(3);
  goal.resize(3);

  start[0] = req.start_pos.x / 1000.0;	//(m)
  start[1] = req.start_pos.y / 1000.0;	//(m)
  start[2] = deg2rad(req.start_pos.th); //(rad)

  goal[0] = req.goal_pos.x / 1000.0; //(m)
  goal[1] = req.goal_pos.y / 1000.0; //(m)
  goal[2] = deg2rad(req.goal_pos.th); //(rad)

  ROS_INFO("Init map ...");

  ROS_INFO("Calculate voronoi path...");

  res.success = SetCollisionArea(static_map_, collision_threshold, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = SetVoronoiLine(static_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = CalcDistFromVoronoi(static_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = ConnectToVoronoi(static_map_, start, res.message);
  if(!res.success){
    ROS_ERROR("Error : Start Point");
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = CalcDistFromVoronoi(static_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = ConnectToVoronoi(static_map_, goal, res.message);
  if(!res.success){
    ROS_ERROR("Error : Goal Point");
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = CalcDistFromVoronoi(static_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = CalcDistFromGoal(static_map_, goal, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  vector<vector<double> > voronoi_path, smooth_path, comp_path;

  res.success = CalcVoronoiPath(static_map_, start, goal, voronoi_path, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  SmoothVoronoiPath(static_map_, start, goal, voronoi_path, smooth_path, kSmoothVoronoiPathThreshold_/1000.0);
  //~ compVoronoiPath(smooth_path, comp_path);

  //~ t_end = clock();
  //~ printf("calc_time:%.2f(sec)\n",(double)(t_end-t_start)/CLOCKS_PER_SEC);

  tms_msg_rp::rps_position temp_pos;
  tms_msg_rp::rps_route rps_robot_path;
  rps_robot_path.robot_id = req.robot_id;
  rps_robot_path.rps_route.clear();

  for(unsigned int i=0;i<smooth_path.size();i++){
    temp_pos.x = smooth_path[i][0] * 1000.0;
    temp_pos.y = smooth_path[i][1] * 1000.0;
    temp_pos.th = rad2deg(smooth_path[i][2]);

    res.VoronoiPath.push_back(temp_pos);
    res.voronoi_path.rps_route.push_back(temp_pos);

    temp_pos.th = smooth_path[i][2];
    rps_robot_path.rps_route.push_back(temp_pos);
  }

  ROS_INFO("...Voronoi Path Plan Success");

  int k = 0;
  while(k<100){
    robot_path_pub_.publish(rps_robot_path);
    k++;
  }
  ROS_INFO("path publish");

  //~ int a;
  //~ scanf("%d", &a);

  res.message = "Voronoi Path Plan Success";
  res.success = 1;
  return true;
}

//------------------------------------------------------------------------------
double TmsRpPathPlanning::GetRobotCollisionThreshold(int robot_id){
  double collision_threshold = 0.0;

  switch(robot_id){
    case 2001:  //SmartPal4
      collision_threshold = kSmartPal4CollisionThreshold_ / 1000.0;
      break;
    case 2002:  //SmartPal5_1
      collision_threshold = kSmartPal5CollisionThreshold_ / 1000.0;
      break;
    case 2003:  //SmartPal5_2
      collision_threshold = kSmartPal5CollisionThreshold_ / 1000.0;
      break;
    case 2004:  //turtlebot2
      collision_threshold = kKobukiCollisionThreshold_ / 1000.0;
      break;
    case 2005:  //kobuki
      collision_threshold = kKobukiCollisionThreshold_ / 1000.0;
      break;
    case 2006:  //KKP
      collision_threshold = kKKPCollisionThreshold_ / 1000.0;
      break;
    default:
      collision_threshold = 0.1;
      break;
  }

  return collision_threshold;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::SetCollisionArea(vector<vector<CollisionMapData> >& map, double threshold, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(unsigned int x=0;x<map.size();x++){
    for(unsigned int y=0;y<map[x].size();y++){
      if(map[x][y].dist_from_obj-threshold<1.0e-5)
        map[x][y].collision = true;
      else
        map[x][y].collision = false;
    }
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::SetVoronoiLine(vector<vector<CollisionMapData> >& map, string& message){
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
bool TmsRpPathPlanning::CalcDistFromVoronoi(vector<vector<CollisionMapData> >& map, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      if(map[x][y].voronoi&&(!map[x][y].collision))
        map[x][y].dist_from_voronoi = 0.0;
      else
        map[x][y].dist_from_voronoi = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }

  for(int x=1;x<map.size()-1;x++){
    for(int y=1;y<map[x].size()-1;y++){
      if(!map[x][y].collision){
        map[x][y].dist_from_voronoi = min( min( min(map[x-1][y-1].dist_from_voronoi+sqrt(2.0)*cell_size, map[x][y-1].dist_from_voronoi+1.0*cell_size),
                                           min(map[x-1][y+1].dist_from_voronoi+sqrt(2.0)*cell_size, map[x-1][y].dist_from_voronoi+1.0*cell_size)),
                                           map[x][y].dist_from_voronoi);
      }
    }
  }

  for(int x=map.size()-2;x>0;x--){
    for(int y=map[x].size()-2;y>0;y--){
      if(!map[x][y].collision){
        map[x][y].dist_from_voronoi = min( min( min(map[x+1][y].dist_from_voronoi+1.0*cell_size, map[x+1][y-1].dist_from_voronoi+sqrt(2.0)*cell_size),
                                                min(map[x][y+1].dist_from_voronoi+1.0*cell_size, map[x+1][y+1].dist_from_voronoi+sqrt(2.0)*cell_size)),
                                                map[x][y].dist_from_voronoi);
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::ConnectToVoronoi(vector<vector<CollisionMapData> >& map, vector<double> connect_point, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  if((connect_point[0]<x_llimit)||(connect_point[0]>x_ulimit)||(connect_point[1]<y_llimit)||(connect_point[1]>y_ulimit)){
    message = "Error : Connect point is out of range";
    cout<<message<<endl;
    cout<<"	x:"<<connect_point[0]<<"	y:"<<connect_point[1]<<"	limit_x:"<<x_llimit<<" ~ "<<x_ulimit<<"	limit_y:"<<y_llimit<<" ~ "<<y_ulimit<<endl;
    return false;
  }

  int target_x, target_y, temp_x, temp_y, dx=0, dy=0;
  target_x = (int)round( (connect_point[0] - x_llimit) / cell_size );
  target_y = (int)round( (connect_point[1] - y_llimit) / cell_size );
  temp_x = target_x;
  temp_y = target_y;

  double temp_dist = map[temp_x][temp_y].dist_from_voronoi;

  if(map[temp_x][temp_y].collision){
    message = "Error : Connect point is collision";
    cout<<message<<endl;
    cout<<"	x:"<<temp_x<<"	y:"<<temp_y<<endl;
    return false;
  }

  if(temp_dist==0.0)
  return true;

  while(temp_dist!=0.0){
    if( map[temp_x-1][temp_y].dist_from_voronoi < temp_dist ){
      dx = -1;
      dy = 0;
      temp_dist = map[temp_x+dx][temp_y].dist_from_voronoi;
    }

    if( map[temp_x][temp_y-1].dist_from_voronoi < temp_dist ){
      dx = 0;
      dy = -1;
      temp_dist = map[temp_x][temp_y+dy].dist_from_voronoi;
    }

    if( map[temp_x+1][temp_y].dist_from_voronoi < temp_dist ){
      dx = 1;
      dy = 0;
      temp_dist = map[temp_x+dx][temp_y].dist_from_voronoi;
    }

    if( map[temp_x][temp_y+1].dist_from_voronoi < temp_dist ){
      dx = 0;
      dy = 1;
      temp_dist = map[temp_x][temp_y+dy].dist_from_voronoi;
    }

    if( map[temp_x-1][temp_y-1].dist_from_voronoi < temp_dist ){
      dx = -1;
      dy = -1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }

    if( map[temp_x-1][temp_y+1].dist_from_voronoi < temp_dist ){
      dx = -1;
      dy = 1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }

    if( map[temp_x+1][temp_y-1].dist_from_voronoi < temp_dist ){
      dx = 1;
      dy = -1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi;
    }

    if( map[temp_x+1][temp_y+1].dist_from_voronoi < temp_dist ){
      dx = 1;
      dy = 1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi;
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
    map[temp_x][temp_y].voronoi = true;
    temp_norm += 1.0;
  }

  temp_x = (int)round(target_x + vec_norm*vec[0]);
  temp_y = (int)round(target_y + vec_norm*vec[1]);
  map[temp_x][temp_y].voronoi = true;

  map[target_x][target_y].voronoi = true;

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::CalcDistFromGoal(vector<vector<CollisionMapData> >& map, vector<double> goal_point, string& message){
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  if((goal_point[0]<x_llimit)||(goal_point[0]>x_ulimit)||(goal_point[1]<y_llimit)||(goal_point[1]>y_ulimit)){
    message = "Error : Goal is out of range";
    cout<<message<<endl;
    return false;
  }

  int temp_x, temp_y;
  temp_x = (int)round( (goal_point[0] - x_llimit) / cell_size );
  temp_y = (int)round( (goal_point[1] - y_llimit) / cell_size );

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      map[x][y].dist_from_goal = (x_ulimit-x_llimit)*(y_ulimit-y_llimit);
    }
  }

  map[temp_x][temp_y].dist_from_goal = 0.0;

  bool change_flg = true;
  double temp = 0.0;

  while(change_flg){
    change_flg = false;

    for(unsigned int i=1;i<map.size()-1;i++){
      for(unsigned int j=1;j<map[i].size()-1;j++){
        if(map[i][j].voronoi){
          temp = map[i][j].dist_from_goal;
          map[i][j].dist_from_goal = min( min( min(map[i-1][j-1].dist_from_goal+sqrt(2.0)*cell_size, map[i-1][j].dist_from_goal+1.0*cell_size),
                                                   map[i-1][j+1].dist_from_goal+sqrt(2.0)*cell_size ),
                                          min( map[i][j-1].dist_from_goal+1.0*cell_size, map[i][j].dist_from_goal ) );
          if(change_flg==false){
          if(map[i][j].dist_from_goal!=temp)
            change_flg=true;
          }
        }
      }
    }

    for(int i=map.size()-2;i>0;i--){
      for(int j=map[i].size()-2;j>0;j--){
        if(map[i][j].voronoi){
          temp = map[i][j].dist_from_goal;
          map[i][j].dist_from_goal = min( min( map[i][j].dist_from_goal, map[i][j+1].dist_from_goal+1.0*cell_size ),
                                          min( map[i+1][j-1].dist_from_goal+sqrt(2.0)*cell_size,
                                                   min( map[i+1][j].dist_from_goal+1.0*cell_size, map[i+1][j+1].dist_from_goal+sqrt(2.0)*cell_size ) ) );
          if(change_flg==false){
            if(map[i][j].dist_from_goal!=temp)
            change_flg=true;
          }
        }
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::CalcVoronoiPath(vector<vector<CollisionMapData> >& map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path, string& message){
  out_path.clear();

  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
  }

  if((start[0]<x_llimit)||(start[0]>x_ulimit)||(start[1]<y_llimit)||(start[1]>y_ulimit)){
    message = "Error : Start point is out of range";
    cout<<message<<endl;
    cout<<"	x:"<<start[0]<<"	y:"<<start[1]<<"	limit_x:"<<x_llimit<<" ~ "<<x_ulimit<<"	limit_y:"<<y_llimit<<" ~ "<<y_ulimit<<endl;
    return false;
  }

  if((goal[0]<x_llimit)||(goal[0]>x_ulimit)||(goal[1]<y_llimit)||(goal[1]>y_ulimit)){
    message = "Error : Goal point is out of range";
    cout<<"	x:"<<goal[0]<<"	y:"<<goal[1]<<"	limit_x:"<<x_llimit<<" ~ "<<x_ulimit<<"	limit_y:"<<y_llimit<<" ~ "<<y_ulimit<<endl;
    cout<<message<<endl;
    return false;
  }

  int i_start_x, i_start_y, i_goal_x, i_goal_y, i_temp_x, i_temp_y, dx=0, dy=0;
  i_start_x = (int)round( (start[0] - x_llimit) / cell_size );
  i_start_y = (int)round( (start[1] - y_llimit) / cell_size );
  i_goal_x = (int)round( (goal[0] - x_llimit) / cell_size );
  i_goal_y = (int)round( (goal[1] - y_llimit) / cell_size );

  if(map[i_start_x][i_start_y].collision){
    message = "Error : Start is collision";
    cout<<message<<endl;
    cout<<"	x:"<<i_start_x<<"	y:"<<i_start_y<<endl;
    return false;
  }

  if(map[i_goal_x][i_goal_y].collision){
    message = "Error : Goal is collision";
    cout<<message<<endl;
    cout<<"	x:"<<i_goal_x<<"	y:"<<i_goal_y<<endl;
    return false;
  }

  map[i_start_x][i_start_y].path = true;
  map[i_goal_x][i_goal_y].path = true;

  //~ out_path.push_back(start);

  i_temp_x = i_start_x;
  i_temp_y = i_start_y;
  double temp_dist = map[i_temp_x][i_temp_y].dist_from_goal;
  vector<double> temp_pos;
  temp_pos.resize(2);

  if(temp_dist==0)
  return true;

  while(1){
    if(temp_dist > map[i_temp_x-1][i_temp_y].dist_from_goal){
      dx = -1;
      dy = 0;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x][i_temp_y-1].dist_from_goal){
      dx = 0;
      dy = -1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x+1][i_temp_y].dist_from_goal){
      dx = 1;
      dy = 0;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x][i_temp_y+1].dist_from_goal){
      dx = 0;
      dy = 1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x-1][i_temp_y-1].dist_from_goal){
      dx = -1;
      dy = -1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x-1][i_temp_y+1].dist_from_goal){
      dx = -1;
      dy = 1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x+1][i_temp_y-1].dist_from_goal){
      dx = 1;
      dy = -1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    if(temp_dist > map[i_temp_x+1][i_temp_y+1].dist_from_goal){
      dx = 1;
      dy = 1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal;
    }

    i_temp_x += dx;
    i_temp_y += dy;
    map[i_temp_x][i_temp_y].path = true;

    if(temp_dist==0.0)
    break;

    temp_pos[0] = (i_temp_x * cell_size) + x_llimit;
    temp_pos[1] = (i_temp_y * cell_size) + y_llimit;
    out_path.push_back(temp_pos);
  }

  //~ out_path.push_back(goal);

  //~ for(int i=0;i<out_path.size();i++){
  //~ cout<<out_path[i][0]<<"	"<<out_path[i][1]<<endl;
  //~ }
  //~ cout<<endl;
  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::SmoothVoronoiPath(vector<vector<CollisionMapData> >& map, vector<double> start, vector<double> goal, vector<vector<double> > in_path, vector<vector<double> >& out_path, double threshold){
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
      if(map[i_temp_x][i_temp_y].dist_from_voronoi>threshold){
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
  //~ cout<<out_path[i][0]<<"	"<<out_path[i][1]<<"	"<<rad2deg(out_path[i][2])<<endl;
  //~ }

  return true;
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
