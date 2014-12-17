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
                                        kMimamorukunCollisionThreshold_(400.0),
                                        kRobotControlWagonDist_(400.0),
                                        kSmoothVoronoiPathThreshold_(100.0),
                                        kPushWagonPathThreshold_(200.0),
                                        ToolBar("TmsRpPathPlanning"),
                                        os_(MessageView::mainInstance()->cout()),
                                        trc_(*TmsRpController::instance())
{
  static ros::NodeHandle nh;

  dynamic_map_sub_ = nh.subscribe("/rps_dynamic_map", 10, &TmsRpPathPlanning::getDynamicMap, this);
  service_voronoi_path_ = nh.advertiseService("/rps_voronoi_path_planning", &TmsRpPathPlanning::voronoiPathPlanner, this);
  robot_path_pub_ = nh.advertise<tms_msg_rp::rps_route>("/rps_robot_path", 1);
}

//------------------------------------------------------------------------------
TmsRpPathPlanning::~TmsRpPathPlanning()
{
}

//------------------------------------------------------------------------------
void TmsRpPathPlanning::getDynamicMap(const tms_msg_rp::rps_map_full::ConstPtr& original_map)
{
  dynamic_map_.clear();

  x_llimit_  = original_map->x_llimit;
  x_ulimit_  = original_map->x_ulimit;
  y_llimit_  = original_map->y_llimit;
  y_ulimit_  = original_map->y_ulimit;
  cell_size_ = original_map->cell_size;

  vector<CollisionMapData> temp_map_line;
  CollisionMapData temp_map_d;

  for(unsigned int x=0;x<original_map->rps_map_x.size();x++){
    temp_map_line.clear();
    for(unsigned int y=0;y<original_map->rps_map_x[x].rps_map_y.size();y++){
      temp_map_d.object_         = original_map->rps_map_x[x].rps_map_y[y].object;
      temp_map_d.collision_      = original_map->rps_map_x[x].rps_map_y[y].object;
      temp_map_d.voronoi_        = original_map->rps_map_x[x].rps_map_y[y].voronoi;
      temp_map_d.dist_from_obj_	 = original_map->rps_map_x[x].rps_map_y[y].dist_from_obj_f;

      temp_map_line.push_back(temp_map_d);
    }
    dynamic_map_.push_back(temp_map_line);
  }
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::voronoiPathPlanner(tms_msg_rp::rps_voronoi_path_planning::Request& req,
                                           tms_msg_rp::rps_voronoi_path_planning::Response& res)
{
  res.success = 0;
  res.VoronoiPath.clear();
  res.voronoi_path.rps_route.clear();
  ROS_INFO("Start Voronoi Path Plan...");
  //~ clock_t t_start,t_end;
  //~ t_start = clock();

  double collision_threshold = getRobotCollisionThreshold(req.robot_id);

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

  ROS_INFO("Calculate voronoi_ path...");

  res.success = setCollisionArea(dynamic_map_, collision_threshold, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = setVoronoiLine(dynamic_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = calcDistFromVoronoi(dynamic_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = connectToVoronoi(dynamic_map_, start, res.message);
  if(!res.success){
    ROS_ERROR("Error : Start Point");
    ROS_ERROR((res.message).c_str());
    return false;
  }
  res.success = calcDistFromVoronoi(dynamic_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = connectToVoronoi(dynamic_map_, goal, res.message);
  if(!res.success){
    ROS_ERROR("Error : Goal Point");
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = calcDistFromVoronoi(dynamic_map_, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  res.success = calcDistFromGoal(dynamic_map_, goal, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  vector<vector<double> > voronoi_path, smooth_path, comp_path;

  res.success = calcVoronoiPath(dynamic_map_, start, goal, voronoi_path, res.message);
  if(!res.success){
    ROS_ERROR((res.message).c_str());
    return false;
  }

  smoothVoronoiPath(dynamic_map_, start, goal, voronoi_path, smooth_path, kSmoothVoronoiPathThreshold_/1000.0);
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
double TmsRpPathPlanning::getRobotCollisionThreshold(int robot_id)
{
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
    case 2007:
      collision_threshold = kMimamorukunCollisionThreshold_ / 1000.0;
      break;
    default:
      collision_threshold = 0.1;
      break;
  }

  return collision_threshold;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::setCollisionArea(vector<vector<CollisionMapData> >& map, double threshold, string& message)
{
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(unsigned int x=0;x<map.size();x++){
    for(unsigned int y=0;y<map[x].size();y++){
      if(map[x][y].dist_from_obj_-threshold<1.0e-5)
        map[x][y].collision_ = true;
      else
        map[x][y].collision_ = false;
    }
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::setVoronoiLine(vector<vector<CollisionMapData> >& map, string& message)
{
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
bool TmsRpPathPlanning::calcDistFromVoronoi(vector<vector<CollisionMapData> >& map, string& message)
{
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      if(map[x][y].voronoi_&&(!map[x][y].collision_))
        map[x][y].dist_from_voronoi_ = 0.0;
      else
        map[x][y].dist_from_voronoi_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
    }
  }

  for(int x=1;x<map.size()-1;x++){
    for(int y=1;y<map[x].size()-1;y++){
      if(!map[x][y].collision_){
        map[x][y].dist_from_voronoi_ = min( min( min(map[x-1][y-1].dist_from_voronoi_+sqrt(2.0)*cell_size_, map[x][y-1].dist_from_voronoi_+1.0*cell_size_),
                                           min(map[x-1][y+1].dist_from_voronoi_+sqrt(2.0)*cell_size_, map[x-1][y].dist_from_voronoi_+1.0*cell_size_)),
                                           map[x][y].dist_from_voronoi_);
      }
    }
  }

  for(int x=map.size()-2;x>0;x--){
    for(int y=map[x].size()-2;y>0;y--){
      if(!map[x][y].collision_){
        map[x][y].dist_from_voronoi_ = min( min( min(map[x+1][y].dist_from_voronoi_+1.0*cell_size_, map[x+1][y-1].dist_from_voronoi_+sqrt(2.0)*cell_size_),
                                                min(map[x][y+1].dist_from_voronoi_+1.0*cell_size_, map[x+1][y+1].dist_from_voronoi_+sqrt(2.0)*cell_size_)),
                                                map[x][y].dist_from_voronoi_);
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::connectToVoronoi(vector<vector<CollisionMapData> >& map, vector<double> connect_point, string& message)
{
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  if((connect_point[0]<x_llimit_)||(connect_point[0]>x_ulimit_)||(connect_point[1]<y_llimit_)||(connect_point[1]>y_ulimit_)){
    message = "Error : Connect point is out of range";
    cout<<message<<endl;
    cout<<"	x:"<<connect_point[0]<<"	y:"<<connect_point[1]<<"	limit_x:"<<x_llimit_<<" ~ "<<x_ulimit_<<"	limit_y:"<<y_llimit_<<" ~ "<<y_ulimit_<<endl;
    return false;
  }

  int target_x, target_y, temp_x, temp_y, dx=0, dy=0;
  target_x = (int)round( (connect_point[0] - x_llimit_) / cell_size_ );
  target_y = (int)round( (connect_point[1] - y_llimit_) / cell_size_ );
  temp_x = target_x;
  temp_y = target_y;

  double temp_dist = map[temp_x][temp_y].dist_from_voronoi_;

  if(map[temp_x][temp_y].collision_){
    message = "Error : Connect point is collision";
    cout<<message<<endl;
    cout<<"	x:"<<temp_x<<"	y:"<<temp_y<<endl;
    return false;
  }

  if(temp_dist==0.0)
  return true;

  while(temp_dist!=0.0){
    if( map[temp_x-1][temp_y].dist_from_voronoi_ < temp_dist ){
      dx = -1;
      dy = 0;
      temp_dist = map[temp_x+dx][temp_y].dist_from_voronoi_;
    }

    if( map[temp_x][temp_y-1].dist_from_voronoi_ < temp_dist ){
      dx = 0;
      dy = -1;
      temp_dist = map[temp_x][temp_y+dy].dist_from_voronoi_;
    }

    if( map[temp_x+1][temp_y].dist_from_voronoi_ < temp_dist ){
      dx = 1;
      dy = 0;
      temp_dist = map[temp_x+dx][temp_y].dist_from_voronoi_;
    }

    if( map[temp_x][temp_y+1].dist_from_voronoi_ < temp_dist ){
      dx = 0;
      dy = 1;
      temp_dist = map[temp_x][temp_y+dy].dist_from_voronoi_;
    }

    if( map[temp_x-1][temp_y-1].dist_from_voronoi_ < temp_dist ){
      dx = -1;
      dy = -1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi_;
    }

    if( map[temp_x-1][temp_y+1].dist_from_voronoi_ < temp_dist ){
      dx = -1;
      dy = 1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi_;
    }

    if( map[temp_x+1][temp_y-1].dist_from_voronoi_ < temp_dist ){
      dx = 1;
      dy = -1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi_;
    }

    if( map[temp_x+1][temp_y+1].dist_from_voronoi_ < temp_dist ){
      dx = 1;
      dy = 1;
      temp_dist = map[temp_x+dx][temp_y+dy].dist_from_voronoi_;
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
    map[temp_x][temp_y].voronoi_ = true;
    temp_norm += 1.0;
  }

  temp_x = (int)round(target_x + vec_norm*vec[0]);
  temp_y = (int)round(target_y + vec_norm*vec[1]);
  map[temp_x][temp_y].voronoi_ = true;

  map[target_x][target_y].voronoi_ = true;

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::calcDistFromGoal(vector<vector<CollisionMapData> >& map, vector<double> goal_point, string& message)
{
  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
    return false;
  }

  if((goal_point[0]<x_llimit_)||(goal_point[0]>x_ulimit_)||(goal_point[1]<y_llimit_)||(goal_point[1]>y_ulimit_)){
    message = "Error : Goal is out of range";
    cout<<message<<endl;
    return false;
  }

  int temp_x, temp_y;
  temp_x = (int)round( (goal_point[0] - x_llimit_) / cell_size_ );
  temp_y = (int)round( (goal_point[1] - y_llimit_) / cell_size_ );

  for(int x=0;x<map.size();x++){
    for(int y=0;y<map[x].size();y++){
      map[x][y].dist_from_goal_ = (x_ulimit_-x_llimit_)*(y_ulimit_-y_llimit_);
    }
  }

  map[temp_x][temp_y].dist_from_goal_ = 0.0;

  bool change_flg = true;
  double temp = 0.0;

  while(change_flg){
    change_flg = false;

    for(unsigned int i=1;i<map.size()-1;i++){
      for(unsigned int j=1;j<map[i].size()-1;j++){
        if(map[i][j].voronoi_){
          temp = map[i][j].dist_from_goal_;
          map[i][j].dist_from_goal_ = min( min( min(map[i-1][j-1].dist_from_goal_+sqrt(2.0)*cell_size_, map[i-1][j].dist_from_goal_+1.0*cell_size_),
                                                   map[i-1][j+1].dist_from_goal_+sqrt(2.0)*cell_size_ ),
                                          min( map[i][j-1].dist_from_goal_+1.0*cell_size_, map[i][j].dist_from_goal_ ) );
          if(change_flg==false){
          if(map[i][j].dist_from_goal_!=temp)
            change_flg=true;
          }
        }
      }
    }

    for(int i=map.size()-2;i>0;i--){
      for(int j=map[i].size()-2;j>0;j--){
        if(map[i][j].voronoi_){
          temp = map[i][j].dist_from_goal_;
          map[i][j].dist_from_goal_ = min( min( map[i][j].dist_from_goal_, map[i][j+1].dist_from_goal_+1.0*cell_size_ ),
                                          min( map[i+1][j-1].dist_from_goal_+sqrt(2.0)*cell_size_,
                                                   min( map[i+1][j].dist_from_goal_+1.0*cell_size_, map[i+1][j+1].dist_from_goal_+sqrt(2.0)*cell_size_ ) ) );
          if(change_flg==false){
            if(map[i][j].dist_from_goal_!=temp)
            change_flg=true;
          }
        }
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
bool TmsRpPathPlanning::calcVoronoiPath(vector<vector<CollisionMapData> >& map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path, string& message)
{
  out_path.clear();

  if(map.empty()){
    message = "Error : Map is empty";
    cout<<message<<endl;
  }

  if((start[0]<x_llimit_)||(start[0]>x_ulimit_)||(start[1]<y_llimit_)||(start[1]>y_ulimit_)){
    message = "Error : Start point is out of range";
    cout<<message<<endl;
    cout<<"	x:"<<start[0]<<"	y:"<<start[1]<<"	limit_x:"<<x_llimit_<<" ~ "<<x_ulimit_<<"	limit_y:"<<y_llimit_<<" ~ "<<y_ulimit_<<endl;
    return false;
  }

  if((goal[0]<x_llimit_)||(goal[0]>x_ulimit_)||(goal[1]<y_llimit_)||(goal[1]>y_ulimit_)){
    message = "Error : Goal point is out of range";
    cout<<"	x:"<<goal[0]<<"	y:"<<goal[1]<<"	limit_x:"<<x_llimit_<<" ~ "<<x_ulimit_<<"	limit_y:"<<y_llimit_<<" ~ "<<y_ulimit_<<endl;
    cout<<message<<endl;
    return false;
  }

  int i_start_x, i_start_y, i_goal_x, i_goal_y, i_temp_x, i_temp_y, dx=0, dy=0;
  i_start_x = (int)round( (start[0] - x_llimit_) / cell_size_ );
  i_start_y = (int)round( (start[1] - y_llimit_) / cell_size_ );
  i_goal_x = (int)round( (goal[0] - x_llimit_) / cell_size_ );
  i_goal_y = (int)round( (goal[1] - y_llimit_) / cell_size_ );

  if(map[i_start_x][i_start_y].collision_){
    message = "Error : Start is collision";
    cout<<message<<endl;
    cout<<"	x:"<<i_start_x<<"	y:"<<i_start_y<<endl;
    return false;
  }

  if(map[i_goal_x][i_goal_y].collision_){
    message = "Error : Goal is collision";
    cout<<message<<endl;
    cout<<"	x:"<<i_goal_x<<"	y:"<<i_goal_y<<endl;
    return false;
  }

  map[i_start_x][i_start_y].path_ = true;
  map[i_goal_x][i_goal_y].path_ = true;

  //~ out_path.push_back(start);

  i_temp_x = i_start_x;
  i_temp_y = i_start_y;
  double temp_dist = map[i_temp_x][i_temp_y].dist_from_goal_;
  vector<double> temp_pos;
  temp_pos.resize(2);

  if(temp_dist==0)
  return true;

  while(1){
    if(temp_dist > map[i_temp_x-1][i_temp_y].dist_from_goal_){
      dx = -1;
      dy = 0;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x][i_temp_y-1].dist_from_goal_){
      dx = 0;
      dy = -1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x+1][i_temp_y].dist_from_goal_){
      dx = 1;
      dy = 0;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x][i_temp_y+1].dist_from_goal_){
      dx = 0;
      dy = 1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x-1][i_temp_y-1].dist_from_goal_){
      dx = -1;
      dy = -1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x-1][i_temp_y+1].dist_from_goal_){
      dx = -1;
      dy = 1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x+1][i_temp_y-1].dist_from_goal_){
      dx = 1;
      dy = -1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    if(temp_dist > map[i_temp_x+1][i_temp_y+1].dist_from_goal_){
      dx = 1;
      dy = 1;
      temp_dist = map[i_temp_x+dx][i_temp_y+dy].dist_from_goal_;
    }

    i_temp_x += dx;
    i_temp_y += dy;
    map[i_temp_x][i_temp_y].path_ = true;

    if(temp_dist==0.0)
    break;

    temp_pos[0] = (i_temp_x * cell_size_) + x_llimit_;
    temp_pos[1] = (i_temp_y * cell_size_) + y_llimit_;
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
bool TmsRpPathPlanning::smoothVoronoiPath(vector<vector<CollisionMapData> >& map, vector<double> start, vector<double> goal, vector<vector<double> > in_path, vector<vector<double> >& out_path, double threshold)
{
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
  i_prev_x = (int)round( (start[0] - x_llimit_) / cell_size_ );
  i_prev_y = (int)round( (start[1] - y_llimit_) / cell_size_ );

  vector<double> vec;
  vec.resize(2);
  double vec_norm, temp_norm;
  bool smoothing = true;

  for(unsigned int i=1;i<in_path.size();i++){
    smoothing = true;
    i_next_x = (int)round( (in_path[i][0] - x_llimit_) / cell_size_ );
    i_next_y = (int)round( (in_path[i][1] - y_llimit_) / cell_size_ );

    vec[0] = i_next_x - i_prev_x;
    vec[1] = i_next_y - i_prev_y;
    vec_norm = sqrt( (vec[0]*vec[0])+(vec[1]*vec[1]) );
    vec[0] /= vec_norm;
    vec[1] /= vec_norm;

    temp_norm = 1.0;
    while(temp_norm<vec_norm){
      i_temp_x = (int)round(i_prev_x + temp_norm*vec[0]);
      i_temp_y = (int)round(i_prev_y + temp_norm*vec[1]);
      if(map[i_temp_x][i_temp_y].dist_from_voronoi_>threshold){
        smoothing = false;
        break;
      }
      temp_norm += 1.0;
    }

    if(!smoothing){
      temp_pos[2] = atan2(i_next_y - i_prev_y, i_next_x - i_prev_x);
      out_path.push_back(temp_pos);
      temp_pos[0] = (i_next_x * cell_size_) + x_llimit_;
      temp_pos[1] = (i_next_y * cell_size_) + y_llimit_;
      out_path.push_back(temp_pos);
      i_prev_x = i_next_x;
      i_prev_y = i_next_y;
    }
  }

  temp_pos[2] = atan2(goal[1] - ((i_prev_y * cell_size_) + y_llimit_), goal[0] - ((i_prev_x * cell_size_) + x_llimit_));
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
