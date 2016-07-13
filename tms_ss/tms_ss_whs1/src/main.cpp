#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <tms_msg_db/TmsdbGetData.h>
#include <tms_msg_db/TmsdbStamped.h>
#include <tms_msg_db/Tmsdb.h>

#define PORT 65001
#define MAX_COUNT 100
#define PEAK 800
#define NOT_PEAK 600
#define FREQ 128.0

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"tms_ss_whs1");
	ros::NodeHandle nh;
	ros::Publisher db_pub;
	db_pub=nh.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data", 1000);

	std::string frame_id("/world");


	int sock;
	int s_len;
	struct sockaddr_in s_address;

	sock=socket(AF_INET,SOCK_DGRAM,0);
	s_address.sin_family=AF_INET;
	s_address.sin_addr.s_addr=INADDR_ANY;
	s_address.sin_port=htons(PORT);

	s_len=sizeof(s_address);
	bind(sock,(struct sockaddr *)&s_address,s_len);

	int count=0;
	int hakei[MAX_COUNT];
	float temp;
	int rate=0;
	bool is_peak=0;
	int rate_count=0;
	while(ros::ok()){
		int rcvmsg=0;
		recvfrom(sock,&rcvmsg,sizeof(&rcvmsg),0,NULL,NULL);

		hakei[count] = (rcvmsg>>16)&0xffff;
		temp = (rcvmsg&0xffff)*0.1;
		ROS_INFO("temp=%f rate=%d wave=%d",temp,rate,hakei[count]);


		if(is_peak==false&&hakei[count]>PEAK){
			is_peak=true;
			rate=(int)(FREQ/(float)rate_count*60.0);
			ROS_INFO("rate_count=%d rate=%d",rate_count,rate);
			rate_count=0;
		}

		if(is_peak==true&&hakei[count]<NOT_PEAK){
			is_peak=false;
		}

		rate_count++;
		count++;

		if(count>=MAX_COUNT){
			count=0;
			char data[500];
			char buf[8];
			const char *c1 = "{";
			strcpy(data,c1);
			strcat(data,"\"temp\":");
			sprintf(buf,"%.1f",temp);
			strcat(data,buf);
			strcat(data,", \"rate\":");
			sprintf(buf,"%d",rate);
			strcat(data,buf);
			strcat(data,", \"wave\":[");
				for(int i=0;i<MAX_COUNT;i++){
					sprintf(buf,"%d",hakei[i]);
					strcat(data,buf);
					if(i!=MAX_COUNT-1) strcat(data,",");
				}
				strcat(data,"]");
				strcat(data,"}");
				ROS_INFO("%s",data);

 				ros::Time now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
				tms_msg_db::TmsdbStamped db_msg;

				db_msg.header.frame_id = frame_id;
				db_msg.header.stamp = now;
				db_msg.tmsdb.clear();
				tms_msg_db::Tmsdb tmpData;

				tmpData.time    = boost::posix_time::to_iso_extended_string(now.toBoost());
				tmpData.id      = 3021;
				tmpData.place   = 5001;
				tmpData.sensor  = 3021;
				tmpData.state   = 1;

				tmpData.note=data;
				db_msg.tmsdb.push_back(tmpData);
				db_pub.publish(db_msg);
			}

		}
		return 0;
	}
