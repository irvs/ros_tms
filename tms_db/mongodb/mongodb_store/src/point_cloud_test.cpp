#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "mongodb_store/message_store.h"

using namespace geometry_msgs;
using namespace mongodb_store;
using namespace std;


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("smaller.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file smaller.pcd \n");
    return (-1);
  }

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from smaller.pcd with the following fields: "
            << std::endl;
  
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  //   std::cout << "    " << cloud->points[i].x
  //             << " "    << cloud->points[i].y
  //             << " "    << cloud->points[i].z << std::endl;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  
  ros::init(argc, argv, "example_mongodb_store_cpp_client");
  ros::NodeHandle nh;

  //Create object which does the work for us.
  MessageStoreProxy messageStore(nh);

  string id(messageStore.insert(msg));

  cout<<"PointCloud2 inserted with id "<<id<<endl;

  vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;

  if(messageStore.queryID<sensor_msgs::PointCloud2>(id, results)) {
    std::cout<<"And got it back too"<<std::endl;
  }

  // the same?
  if(msg.data.size() == results[0]->data.size()) {
   std::cout<<"And sizes match"<<std::endl; 
  }

  if(msg.data[3425] == results[0]->data[3425]) {
   std::cout<<"And some random element matches"<<std::endl; 
  }

  return (0);
}