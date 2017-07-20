#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <people_msgs/People.h>

#define MARGIN 1.0  // Ninebot Width 546mm, Depth 262mm
#define INVALID_POS -999.9999
#define ROT_ANGLE -M_PI / 2

ros::Publisher pub;
double ninebot_x, ninebot_y;

// void cloudCallback(const sensor_msgs::PointCloud &cloud_in){
//
//   // Record indexes within the area
//   std::vector<int> area_index;
//   for(int i = 0; i < cloud_in.points.size(); ++i){
//     float x = cloud_in.points.at(i).x;
//     float y = cloud_in.points.at(i).y;
//
//     if(((ninebot_x - MARGIN) < x) && (x < (ninebot_x + MARGIN)) && ((ninebot_y - MARGIN) < y) && (y < (ninebot_y + MARGIN))){
//       area_index.push_back(i);
//     }
//   }
//
//   sensor_msgs::PointCloud cloud_out = cloud_in;
//   cloud_out.header.frame_id = "map1";
//
//   // Coordinate Transformation
//   for(int i = 0; i < cloud_out.points.size(); ++i){
//     float temp_x = cloud_in.points.at(i).x;
//     float temp_y = cloud_in.points.at(i).y;
//
//     cloud_out.points.at(i).x = temp_x * cos(ROT_ANGLE) - temp_y * sin(ROT_ANGLE);
//     cloud_out.points.at(i).y = temp_x * sin(ROT_ANGLE) + temp_y * cos(ROT_ANGLE);
//   }
//
//   // Delete point (Set invalid value)
//   for(std::vector<int>::iterator itr = area_index.begin(),
//                                  end = area_index.end();
//                                  itr != end; ++itr){
//     int it = (int)*itr;
//     cloud_out.points.at(it).x = INVALID_POS;
//     cloud_out.points.at(it).y = INVALID_POS;
//   }
//
//   pub.publish(cloud_out);
// }

void peopleCallback(const people_msgs::People &people_in){

  // Record indexes within the area
  std::vector<int> area_index;
  for(int i = 0; i < people_in.people.size(); ++i){
    float x = people_in.people.at(i).position.x;
    float y = people_in.people.at(i).position.y;

    if(((ninebot_x - MARGIN) < x) && (x < (ninebot_x + MARGIN)) && ((ninebot_y - MARGIN) < y) && (y < (ninebot_y + MARGIN))){
      area_index.push_back(i);
    }
  }

  people_msgs::People people_out = people_in;
  people_out.header.frame_id = "map1";

  // Coordinate Transformation
  for(int i = 0; i < people_out.people.size(); ++i){
    float temp_x = people_out.people.at(i).position.x;
    float temp_y = people_out.people.at(i).position.y;

    people_out.people.at(i).position.x = temp_x * cos(ROT_ANGLE) - temp_y * sin(ROT_ANGLE);
    people_out.people.at(i).position.y = temp_x * sin(ROT_ANGLE) + temp_y * cos(ROT_ANGLE);
  }

  // Delete point (Set invalid value)
  for(std::vector<int>::iterator itr = area_index.begin(),
                                 end = area_index.end();
                                 itr != end; ++itr){
    int it = (int)*itr;
    people_out.people.at(it).position.x = INVALID_POS;
    people_out.people.at(it).position.y = INVALID_POS;
  }

  pub.publish(people_out);
}

void poseCallback(const geometry_msgs::PoseStamped &ninebot_position){
  // Get Ninebot position
  ninebot_x = ninebot_position.pose.position.x;
  ninebot_y = ninebot_position.pose.position.y;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ninebot_cropper");
  ros::NodeHandle nh;

  //ros::Subscriber cloud_sub = nh.subscribe("cloud_p2sen", 10, cloudCallback);
  ros::Subscriber people_sub = nh.subscribe("people_p2sen", 10, peopleCallback);
  ros::Subscriber pose_sub = nh.subscribe("ninebot_pos", 10, poseCallback);

  //pub = nh.advertise<sensor_msgs::PointCloud>("cloud_ninebot_cropped", 10);
  pub = nh.advertise<people_msgs::People>("people_ninebot_cropped", 10);

  ros::spin();
  return 0;
}
