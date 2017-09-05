#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <people_msgs/People.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#define MARGIN 0.6  // Ninebot Width 546mm, Depth 262mm

bool GetInitPosition = false;
ros::Publisher pub, pub_pot, pub_vis, pub_vis_array;
double ninebot_x, ninebot_y, ninebot_z, ninebot_ox, ninebot_oy, ninebot_oz, ninebot_ow;

void peopleCallback(const people_msgs::People &people_in){
  float mindist = FLT_MAX;
  nav_msgs::Odometry ninebot_measured_position;

  people_msgs::People people_out;
  people_msgs::Person person;
  people_out.header.frame_id = "/map";

  for(int i = 0; i < people_in.people.size(); ++i){
    float x = people_in.people.at(i).position.x;
    float y = people_in.people.at(i).position.y;
    float dist = sqrt((ninebot_x-x)*(ninebot_x-x)+(ninebot_y-y)*(ninebot_y-y));
    if(GetInitPosition && dist < MARGIN){
      if(mindist>dist){
        mindist = dist;
        ninebot_measured_position.header.frame_id = "/map";
        ninebot_measured_position.header.stamp = ros::Time::now();
        ninebot_measured_position.pose.pose.position.x = x;
        ninebot_measured_position.pose.pose.position.y = y;
        ninebot_measured_position.pose.pose.position.z = 0;
        ninebot_measured_position.pose.pose.orientation.x = 0; 
        ninebot_measured_position.pose.pose.orientation.y = 0;
        ninebot_measured_position.pose.pose.orientation.z = 0; 
        ninebot_measured_position.pose.pose.orientation.w = 1;
      }
    } else {
      person.position.x = people_in.people.at(i).position.x;
      person.position.y = people_in.people.at(i).position.y;
      person.position.z = people_in.people.at(i).position.z;
      people_out.people.push_back(person);
    }
  }

  pub.publish(people_out);

  if(mindist<MARGIN) pub_pot.publish(ninebot_measured_position);

  visualization_msgs::MarkerArray markerArray;
  uint32_t shape_shpere = visualization_msgs::Marker::SPHERE;
  uint32_t shape_mesh = visualization_msgs::Marker::MESH_RESOURCE;
  int id = 1;

  for(int i = 0; i < people_out.people.size(); ++i){
    visualization_msgs::Marker marker_object;
    marker_object.header.frame_id = "/world_link";
    marker_object.header.stamp = ros::Time::now();

    marker_object.id = id++;
    //marker_object.type = shape_shpere;
    marker_object.type = shape_mesh;

    marker_object.action = visualization_msgs::Marker::ADD;

    if(marker_object.type == shape_mesh){
      marker_object.mesh_resource = "package://tms_ss_pot/meshes/WalkingMan4.dae";
      marker_object.mesh_use_embedded_materials = true;
      marker_object.scale.x = 0.025;
      marker_object.scale.y = 0.025;
      marker_object.scale.z = 0.025;
      marker_object.pose.position.z = 0.0;
    } else {
      marker_object.color.r = 0.0;
      marker_object.color.g = 1.0;
      marker_object.color.b = 0.0;
      marker_object.color.a = 1.0;
      marker_object.scale.x = 1.0;
      marker_object.scale.y = 1.0;
      marker_object.scale.z = 1.0;
      marker_object.pose.position.z = 0.5;
    }
    marker_object.pose.position.x = people_out.people.at(i).position.x;
    marker_object.pose.position.y = people_out.people.at(i).position.y;
    marker_object.pose.orientation.x = 0;
    marker_object.pose.orientation.y = 0;
    marker_object.pose.orientation.z = 0;
    marker_object.pose.orientation.w = 1;

    marker_object.lifetime = ros::Duration(0.5);

    markerArray.markers.push_back(marker_object);
  }

  // Green ball is measured ninebot's position
#if 0
  if(mindist < MARGIN){
    visualization_msgs::Marker marker_object;
    marker_object.header.frame_id = "/world_link";
    marker_object.header.stamp = ros::Time::now();

    marker_object.id = id++;
    marker_object.type = shape_shpere;
    //marker_object.type = shape_mesh;

    marker_object.action = visualization_msgs::Marker::ADD;

    if(marker_object.type == shape_mesh){
      marker_object.mesh_resource = "package://tms_ss_pot/meshes/WalkingMan4.dae";
      marker_object.mesh_use_embedded_materials = true;
      marker_object.scale.x = 0.025;
      marker_object.scale.y = 0.025;
      marker_object.scale.z = 0.025;
      marker_object.pose.position.z = 0.0;
    } else {
      marker_object.color.r = 0.0;
      marker_object.color.g = 1.0;
      marker_object.color.b = 0.0;
      marker_object.color.a = 1.0;
      marker_object.scale.x = 1.0;
      marker_object.scale.y = 1.0;
      marker_object.scale.z = 1.0;
      marker_object.pose.position.z = 0.5;
    }
    marker_object.pose.position.x = ninebot_measured_position.pose.pose.position.x;
    marker_object.pose.position.y = ninebot_measured_position.pose.pose.position.y;
    marker_object.pose.orientation.x = 0;
    marker_object.pose.orientation.y = 0;
    marker_object.pose.orientation.z = 0;
    marker_object.pose.orientation.w = 1;

    marker_object.lifetime = ros::Duration(0.5);

    markerArray.markers.push_back(marker_object);
  }
#endif

  pub_vis_array.publish(markerArray);

  // Red ball is ninebot's position by odom after applying kalman filter
  if(GetInitPosition){
    visualization_msgs::Marker marker_ninebot;
    marker_ninebot.header.frame_id = "/world_link";
    marker_ninebot.header.stamp = ros::Time::now();

    marker_ninebot.id = 1;
    //marker_ninebot.type = shape_shpere;
    marker_ninebot.type = shape_mesh;

    marker_ninebot.action = visualization_msgs::Marker::ADD;

    if(marker_ninebot.type == shape_mesh){
      //marker_ninebot.mesh_resource = "package://tms_ss_pot/meshes/WalkingMan4.dae";
      marker_ninebot.mesh_resource = "package://tms_ss_pot/meshes/ninebot_v2.dae";

      marker_ninebot.mesh_use_embedded_materials = true;
      marker_ninebot.scale.x = 0.01;
      marker_ninebot.scale.y = 0.01;
      marker_ninebot.scale.z = 0.01;
      marker_ninebot.pose.position.z = 0.0;
    } else {
      marker_ninebot.color.r = 1.0;
      marker_ninebot.color.g = 0.0;
      marker_ninebot.color.b = 0.0;
      marker_ninebot.color.a = 1.0;
      marker_ninebot.scale.x = 1.0;
      marker_ninebot.scale.y = 1.0;
      marker_ninebot.scale.z = 1.0;
      marker_ninebot.pose.position.z = 0.5;
    }

    marker_ninebot.pose.position.x = ninebot_x;
    marker_ninebot.pose.position.y = ninebot_y;
    marker_ninebot.pose.orientation.x = ninebot_ox;
    marker_ninebot.pose.orientation.y = ninebot_oy;
    marker_ninebot.pose.orientation.z = ninebot_oz;
    marker_ninebot.pose.orientation.w = ninebot_ow;

    marker_ninebot.lifetime = ros::Duration(0.5);

    pub_vis.publish(marker_ninebot);
  }

}

void odomCallback(const nav_msgs::Odometry &odom_position){
  // Get Ninebot position
  ninebot_x = odom_position.pose.pose.position.x;
  ninebot_y = odom_position.pose.pose.position.y;
  ninebot_z = odom_position.pose.pose.position.z;
  ninebot_ox = odom_position.pose.pose.orientation.x;
  ninebot_oy = odom_position.pose.pose.orientation.y;
  ninebot_oz = odom_position.pose.pose.orientation.z;
  ninebot_ow = odom_position.pose.pose.orientation.w;
  GetInitPosition = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ninebot_cropper");
  ros::NodeHandle nh;

  ros::Subscriber people_sub = nh.subscribe("people_p2sen", 10, peopleCallback);
  ros::Subscriber odom_sub = nh.subscribe("/portable1/odom", 10, odomCallback);

  pub = nh.advertise<people_msgs::People>("people_ninebot_cropped", 10);
  pub_pot = nh.advertise<nav_msgs::Odometry>("ninebot_measured_pos", 10);
  pub_vis = nh.advertise< visualization_msgs::Marker >("visualization_ninebot", 1);
  pub_vis_array = nh.advertise< visualization_msgs::MarkerArray >("visualization_objects", 1);

  ros::spin();
  return 0;
}
