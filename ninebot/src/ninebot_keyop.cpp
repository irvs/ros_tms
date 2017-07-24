//include
#include "ros/ros.h"
#include "math.h"
#include "iostream"
#include "string"
#include "geometry_msgs/Twist.h"


#define ANGLE_CHANGE 0.01


//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "portable_keyop");
  ros::NodeHandle n;
  
  ros::Publisher key_pub = n.advertise<geometry_msgs::Twist>("portable1/serial_twist",100);
  
  geometry_msgs::Twist key_vel;

  double angle_vel  = 0.0;

  ros::Rate loop(10);
  while(ros::ok())
  {
    char command_c;       //char
    std::string command;  //string
    std::cout << "angle_vel " << angle_vel << std::endl;

    //input char
    std::cin >> command_c;
    switch(command_c)
    {
        case 'a':
            angle_vel = angle_vel + ANGLE_CHANGE; 
            break;
        //turn right
        case 'd':
            angle_vel = angle_vel - ANGLE_CHANGE; 
            break;
        //stop
        case 's':
            angle_vel = 0.0;
            break;
        //ros::shutdown
        case 'c':
            ros::shutdown(); 
            break;
        default:
            break;
    }
    //input pub data
    key_vel.angular.z = angle_vel;
    key_pub.publish(key_vel);
    loop.sleep();
  }  
  return 0;
}
