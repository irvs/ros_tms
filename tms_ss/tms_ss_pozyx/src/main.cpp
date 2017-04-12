#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

#define DEVNAME "/dev/ttyACM2"
#define BAUDRATE B115200
#define BUFSIZE 256

#define PI 3.14159265
#define DEG2RAD(x) ((x)*(PI/180))

using namespace std;
using namespace boost;


int main(int argc, char **argv)
{
  ros::init(argc,argv,"tms_ss_pozyx");
  ros::NodeHandle n;
  ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("pozyx",1000);

	int fd = open(DEVNAME, O_RDWR | O_NOCTTY);
	if(fd<0){
		ROS_ERROR("cannot open device");
		return 0;
	}
	ROS_INFO("device opened");
	struct termios tio;
	memset(&tio,0,sizeof(tio));
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME]=0;
	tio.c_lflag=ICANON;
	tio.c_iflag=IGNPAR | ICRNL;
	cfsetspeed(&tio,BAUDRATE);
	tcsetattr(fd,TCSANOW,&tio);

	int len;
	char buf[BUFSIZE];
	while(ros::ok()){
		memset(&buf,0,sizeof(buf));
		len = read(fd,buf,BUFSIZE);
		if(len==0)
			continue;
	  std::string data(buf);
	  std::vector<std::string> v_data;
	  v_data.clear();
    boost::split(v_data,data,boost::is_any_of(","));
    if(v_data.size()==7){    
      int id = atoi(v_data.at(0).c_str());
      double X = 0.001*atoi(v_data.at(1).c_str());
      double Y = 0.001*atoi(v_data.at(2).c_str());
      double Z = 0.001*atoi(v_data.at(3).c_str());
      double rr = atof(v_data.at(4).c_str());
      double rp = atof(v_data.at(5).c_str());
      double ry = atof(v_data.at(6).c_str());
      tf::Quaternion q = tf::createQuaternionFromRPY(rr,rp,ry);

      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = "world_link";
      msg.header.stamp = ros::Time::now();
      msg.pose.position.x = X;
      msg.pose.position.y = Y;
      msg.pose.position.z = 1.3;
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      pos_pub.publish(msg);
    }
  }
  return 0;
}
