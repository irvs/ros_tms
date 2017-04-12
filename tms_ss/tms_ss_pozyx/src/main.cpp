#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "kalman-Ndof.hpp"
#include <curl/curl.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_datatypes.h>
#include <math.h>

#define PI 3.14159265
#define DEG2RAD(x) ((x)*(PI/180))
static int sofd;
static struct hostent *shost;
static struct sockaddr_in sv_addr;

using namespace std;
using namespace boost;


class SimpleKalman
{
	const static double Q = 0.001;//0.0001;
	const static double R = 0.1;//0.01;
	double P, X, K;

public:
	SimpleKalman()
		:P(0.0), X(0.0)
		 {};

	double update(double measurement)
	{
		measurementUpdate();
		double result = X + (measurement - X) * K;
		X = result;
		return result;
	}

private:
	void measurementUpdate()
	{
		K = (P + Q) / (P + Q + R);
		P = R * (P + Q) / (R + P + Q);
	}
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"tms_ss_pozyx");
  ros::NodeHandle n;
  ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("pozyx",1000);

      SimpleKalman sk_X,sk_Y,sk_Z;
  // Kalman kalman(6,6,3);
  ros::Time last_time;
  ros::Time now;

  char target_uri[1024];
  char http_res[1024];
  char target_host[256];
  int target_portno = 80;
  strcpy(target_host,HOST);
  printf("host:%s\n",target_host);
  strcpy(target_uri,URI);
  printf("uri:%s\n",target_uri);
  shost = gethostbyname(HOST);
  memset(&sv_addr,'\0',sizeof(sv_addr));
  sv_addr.sin_family=AF_INET;
  sv_addr.sin_port = htons(target_portno);
  memcpy((char *)&sv_addr.sin_addr, (char *)shost->h_addr,shost->h_length);

  int value[6];
  // int X=0;
  // int Y=0;
  // int Z=0;
  // int preX,preY,preZ;

  ros::Rate loop_rate(10);
  now = ros::Time::now();
  while(ros::ok())
  {
  sofd = socket(AF_INET,SOCK_STREAM,0);
    if(connect(sofd,(const struct sockaddr*)&sv_addr,sizeof(sv_addr))<0){
      printf("cannot connect\n");
    }else{
      printf("connect\n");
      usleep(50000);
    send(sofd, "GET ",      strlen("GET "),      0);
    send(sofd, target_uri,  strlen(target_uri),  0);
    send(sofd, " HTTP/1.0", strlen(" HTTP/1.0"), 0);
    send(sofd, "\r\n",      strlen("\r\n"),      0);
    send(sofd, "Host: ",    strlen("Host: "),    0);
    send(sofd, target_host, strlen(target_host), 0);
    send(sofd, "\r\n",      strlen("\r\n"),      0);
    send(sofd, "\r\n",      strlen("\r\n"),      0);

    printf("recv");
    while (recv(sofd, http_res, sizeof(http_res), 0)) {
    }
    printf("http:%s\n",http_res);

    std::string data(http_res);
    std::vector<std::string> v_data;
    v_data.clear();
    boost::split(v_data,data,boost::is_any_of(","));
    value[1]=atoi(v_data.at(1).c_str());
    value[2]=atoi(v_data.at(2).c_str());
    value[3]=atoi(v_data.at(3).c_str());
    value[4]=atoi(v_data.at(4).c_str());
    value[5]=atoi(v_data.at(5).c_str());
    boost::split(v_data,v_data.at(0),boost::is_any_of(">"));
    value[0]=atoi(v_data.at(2).c_str());
    for(int i=0;i<6;i++){
      printf("value[%d]:%d\n",i,value[i]);
    }

    double X = sk_X.update(value[0]);
    double Y = sk_Y.update(value[1]);
    double Z = sk_Z.update(value[2]);
    double rr = DEG2RAD(value[3]);//sk_rr.update(value[3]);
    double rp = DEG2RAD(value[4]);//sk_rp.update(value[4]);
    double ry = DEG2RAD(value[5]);//sk_ry.update(value[5]);
    ROS_INFO("kf:%f,%f,%f,%f,%f,%f",X,Y,Z,rr,rp,ry);

    // last_time = now;
    // now = ros::Time::now();
    // double delta_t = (now.sec+now.nsec*0.000000001)-(last_time.sec+last_time.nsec*0.000000001);
    // // printf("delta_t:%f",delta_t);
    // preX=X;
    // preY=Y;
    // preZ=Z;
    // X=value[0];
    // Y=value[1];
    // Z=value[2];
    // double velX = (X-preX)/delta_t;
    // double velY = (Y-preY)/delta_t;
    // double velZ = (Z-preZ)/delta_t;
    //
    // static int count = 0;
    //
    // if(count==0){
    //   count++;
    //   continue;
    // }else if(count=1){
    //   kalman.init(0.1,1.0,0.01);
    //   double C[3][6];
    //   memset(C,0,sizeof(C));
    //   C[0][0]=C[1][1]=C[2][2]=1;
    //   kalman.setC((double *)&C);
    //   kalman.setX(0,X);
    //   kalman.setX(1,Y);
    //   kalman.setX(2,Z);
    //   kalman.setX(3,velX);
    //   kalman.setX(4,velY);
    //   kalman.setX(5,velZ);
    //   count++;
    // }
    //
    // double A[3][6];
    // memset(A,0,sizeof(A));
    // A[0][0]=A[1][1]=A[2][2]=1;
    // A[0][3]=A[1][4]=A[2][5]=delta_t;
    // double obs[6]={X,Y,Z,velX,velY,velZ};
    // double input[3]={0,0,0};
    //
    // kalman.update(obs,input);
    //
    // double kfX,kfY,kfZ;
    // kfX = kalman.getX(0);
    // kfY = kalman.getX(1);
    // kfZ = kalman.getX(2);
    //
    // ROS_INFO("kf:%f,%f,%f",kfX,kfY,kfZ);

    tf::Quaternion q = tf::createQuaternionFromRPY(rr,rp,ry);//(value[3],value[4],value[5]);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world_link";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = X * 0.001;
    msg.pose.position.y = Y * 0.001;
    msg.pose.position.z = 1.3;//kfZ * 0.001;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pos_pub.publish(msg);


    // split(v_data,v_data.at(0),boost::is_any_of(">"));
    // printf("v_data(2):%s\n",v_data.at(2).c_str());
    // value[0]=atoi(v_data.at(2).c_str());


    memset(&http_res, '\0', sizeof(http_res));
    close(sofd);
    }
  }
  return 0;
}
