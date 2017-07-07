#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>

#define MARGIN 0.5  // Ninebot Width 546mm, Depth 262mm
#define INVALID_POS -999.9999

ros::Publisher pub;
double ninebot_x, ninebot_y;

void cloudCallback(const sensor_msgs::PointCloud &cloud_in){

  // Record indexes within the area
  std::vector<int> area_index;
  for(int i = 0; i < cloud_in.points.size(); ++i){
    float x = cloud_in.points.at(i).x;
    float y = cloud_in.points.at(i).y;
    //Debug
    // std::cout << "x:" << x << " y:" << y << std::endl;
    ninebot_x = 0.0;
    ninebot_y = 0.0;
    //
    if(((ninebot_x - MARGIN) < x) && (x < (ninebot_x + MARGIN)) && ((ninebot_y - MARGIN) < y) && (y < (ninebot_y + MARGIN))){
      area_index.push_back(i);
    }
  }

  // Delete point (Set invalid value)
  sensor_msgs::PointCloud cloud_out = cloud_in;
  for(std::vector<int>::iterator itr = area_index.begin(),
                                 end = area_index.end();
                                 itr != end; ++itr){
    //std::cout << "itr:" << *itr << std::endl;
    int j = (int)*itr;
    cloud_out.points.at(j).x = INVALID_POS;
    cloud_out.points.at(j).y = INVALID_POS;
  }

  pub.publish(cloud_out);
  ROS_INFO("Data Published");

}

void poseCallback(const geometry_msgs::PoseStamped &ninebot_position){
  // Get Ninebot position
  ninebot_x = ninebot_position.pose.position.x;
  ninebot_y = ninebot_position.pose.position.y;
  //Debug
  std::cout << "ninebot_x: " << ninebot_x << " ninebot_y: " << ninebot_y << std::endl;
}


int main(int argc, char** argv){
  ROS_INFO("Hello");
  ros::init(argc, argv, "ninebot_cropper");
  ros::NodeHandle nh;

  ros::Subscriber cloud_sub = nh.subscribe("cloud_p2sen", 10, cloudCallback);
  ros::Subscriber pose_sub = nh.subscribe("ninebot_pos", 10, poseCallback);

  pub = nh.advertise<sensor_msgs::PointCloud>("cloud_ninebot_cropped", 10);

  ros::spin();
}
