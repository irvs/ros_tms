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
#define DB_WRITE 10
#define PEAK 800
#define NOT_PEAK 600

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"tms_ss_whs1");
	ros::NodeHandle nh;
	ros::Publisher db_pub;
	db_pub=nh.advertise<tms_msg_db::TmsdbStamped> ("tms_db_data", 1000);
	std::string frame_id("/world");

	int sock;
	sock=socket(AF_INET,SOCK_DGRAM,0);
	if(sock<0){
		ROS_ERROR("socket error\n");
		return 0;
	}

	struct sockaddr_in s_address;
	s_address.sin_family=AF_INET;
	s_address.sin_addr.s_addr=INADDR_ANY;
	s_address.sin_port=htons(PORT);
	bind(sock,(struct sockaddr *)&s_address,sizeof(s_address));

	ROS_INFO("tms_ss_whs1 ready...");

	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;

	int count=0;
	int msec=0;
	int hakei[MAX_COUNT];
	float temp;
	float acc_x,acc_y,acc_z;
	double roll,pitch;
	int rate=0,p_rate=0;
	bool is_peak=false;
	int db_count=0;
	int last_peak_time=-1;
	fd_set rfds,rfds2;
	int n;
	while(ros::ok()){
		memcpy(&rfds2,&rfds,sizeof(fd_set));
		n=select(sock+1,&rfds2,NULL,NULL,&tv);
		if(n==0){
			ROS_INFO("timeout");
		}
		else if(FD_ISSET(sock,&rfds2)){
			int rcvmsg[3];
			n = recvfrom(sock,rcvmsg,sizeof(rcvmsg),0,NULL,NULL);
			if(n<0){
				ROS_WARN("recvfrom error");
			}else{
				msec = (rcvmsg[0]>>16)&0xffff;
				hakei[count] = (rcvmsg[0]&0xffff);
				temp = ((rcvmsg[1]>>16)&0xffff)*0.01;
				acc_x = (((short)(rcvmsg[1]&0xffff)<<16)>>16)*0.01;
				acc_y = (rcvmsg[2]>>16)*0.01;
				acc_z = (((short)(rcvmsg[2]&0xffff)<<16)>>16)*0.01;

				if(acc_y == 0.0) roll = 0;
				else roll = atan(acc_x/acc_y);
				double G = sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z);
				if(G == 0.0) pitch = 0;
			  else pitch = asin(-acc_z/G);

				ROS_INFO("rate:%d roll:%f pitch:%f acc_x:%f acc_y:%f acc_z:%f",rate,roll,pitch,acc_x,acc_y,acc_z);

				if(is_peak==false&&hakei[count]>PEAK){
					is_peak=true;
					if(last_peak_time==-1)
					last_peak_time = msec;
					else{
						int interval = msec-last_peak_time;
						if(interval<0)
						interval += 60000;
						p_rate = rate;
						rate = (int)(1000.0 / (double)interval * 60.0);
						if(rate<50) rate = 0;
						else if(rate>200) rate = p_rate;
						last_peak_time = msec;
					}
				}

				if(is_peak==true&&hakei[count]<NOT_PEAK){
					is_peak=false;
				}

				count++;
				db_count++;

				if(db_count>=DB_WRITE){
					db_count=0;
					char data[500];
					char buf[8];
					const char *c1 = "{";
					strcpy(data,c1);
					strcat(data,"\"temp\":");
					sprintf(buf,"%.2f",temp);
					strcat(data,buf);
					strcat(data,", \"rate\":");
					sprintf(buf,"%d",rate);
					strcat(data,buf);
					strcat(data,", \"wave\":[");
					for(int i=0;i<MAX_COUNT;i++){
						int j = ((count+i)>=MAX_COUNT) ? (count+i-MAX_COUNT) : count+i;
						sprintf(buf,"%d",hakei[j]);
						strcat(data,buf);
						if(i!=MAX_COUNT-1) strcat(data,",");
					}
					strcat(data,"]");
					strcat(data,"}");

					ros::Time now = ros::Time::now() + ros::Duration(9*60*60); // GMT +9
					tms_msg_db::TmsdbStamped db_msg;

					db_msg.header.frame_id = frame_id;
					db_msg.header.stamp = now;
					db_msg.tmsdb.clear();
					tms_msg_db::Tmsdb tmpData;

					tmpData.time    = boost::posix_time::to_iso_extended_string(now.toBoost());
					tmpData.name    = "whs1_mybeat";
					tmpData.id      = 3021;
					tmpData.place   = 5001;
					tmpData.sensor  = 3021;
					tmpData.state   = 1;
					tmpData.rr			= roll;
					tmpData.rp 			= pitch;
					tmpData.ry			= 0;

					tmpData.note=data;
					db_msg.tmsdb.push_back(tmpData);
					db_pub.publish(db_msg);
				}

				if(count>=MAX_COUNT){
					count=0;
				}
			}
		}

		ros::spinOnce();
	}
	close(sock);
	return 0;
}
