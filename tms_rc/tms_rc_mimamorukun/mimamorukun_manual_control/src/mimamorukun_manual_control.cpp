//------------------------------------------------------------------------------
// FileName : wc_controller.cpp
// Date		: 2014.06.03
// author 	: Akio Shigekane
//------------------------------------------------------------------------------
//2014. 9.23     	adjust scale of speed
//2014.11.01        adjust wheel spin PID constants

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

#include "ClientSocket.h"
#include "SocketException.h"
#include <iostream>
#include <string>

#include <sensor_msgs/Joy.h>

#define ROS_RATE   10

using namespace std;

ClientSocket  client_socket ( "192.168.11.99", 54300 );
const int     ENC_MAX  = 3932159;
const int     SPEED_MAX = 32767;
const float   DIST_PER_PULSE = 0.552486;  //mm par pulse
const int     WHEEL_DIST = 533;

long int 	ENC_L = 0;
long int 	ENC_R = 0;
int 	POS_X = 0;
int 	POS_Y = 0;
double 	POS_SIGMA = 0;
double 	POS_ANG = 0;

int spd_raw_L   = 0;
int spd_raw_R   = 0;
double spd_mm_L = 0;
double spd_mm_R = 0;
int spd_cmd_L = 0;
int spd_cmd_R = 0;
double joy_cmd_spd = 0.0;
double joy_cmd_turn = 0.0;

class MachinePose_s{
private:
public:
    MachinePose_s() {};
    ~MachinePose_s() {};
    void updateOdom();
    void updateVicon();
    void updateCompFilter();
    geometry_msgs::Pose2D   pos_odom;
    geometry_msgs::Pose2D   pos_vicon;
    geometry_msgs::Pose2D   pos_fusioned;
/*    geometry_msgs::Twist    vel_odom;
    geometry_msgs::Twist    vel_vicon;
    geometry_msgs::Twist    vel_fusioned;*/
    geometry_msgs::Pose2D vel_odom;
    geometry_msgs::Pose2D vel_vicon;
    geometry_msgs::Pose2D vel_fusioned;
}mchn_pose;


int Dist2Pulse(int dist){   return ((float)dist)/DIST_PER_PULSE;}
int Pulse2Dist(int pulse){  return ((float)pulse)*DIST_PER_PULSE;}
double Rad2Deg(double rad){ return rad*(180.0)/M_PI;}
double Deg2Rad(double deg){ return deg*M_PI/180.0;}
double Limit(double val,double max,double min){
    if(val > max)       return max;
    else if(min > val)  return min;
    else                return val;
}

void MachinePose_s::updateOdom(){
    //update Encoder value
    long int tmpENC_L = 0;
    long int tmpENC_R = 0;
    string reply;
    client_socket << "@GP1@GP2";    /*use 250ms for send and get reply*/
/*    client_socket >> reply;          
    cout << "Response:" << reply << "\n";*/
    //sscanf(reply.c_str(),"@GP1,%ld@GP2,%ld",&tmpENC_L,&tmpENC_R);
    if(tmpENC_L > ENC_MAX/2)ENC_L = tmpENC_L-(ENC_MAX+1);
    else                    ENC_L = tmpENC_L;
    if(tmpENC_R > ENC_MAX/2)ENC_R = tmpENC_R-(ENC_MAX+1);
    else                    ENC_R = tmpENC_R;

	static long int ENC_R_old = 0;static long int ENC_L_old = 0;
	double detLp , r , dX ,dY,dL;

	//エンコーダーの位置での前回からの移動距離dL_R,dL_Lを算出
	double dL_L = (double)(ENC_L-ENC_L_old)*(-DIST_PER_PULSE);
    double dL_R = (double)(ENC_R-ENC_R_old)*  DIST_PER_PULSE;

///////////////////////////////////////////////////////////
    ////////////////////for debuggind//////////////
    spd_raw_L = ENC_L-ENC_L_old;
    spd_raw_R = ENC_R-ENC_R_old;
    spd_mm_L = dL_L;
    spd_mm_R = dL_R;

	//角速度SIGMAを算出
	//POS_SIGMA = ((ENC_R-ENC_R_old)*DIST_PER_PULSE - (ENC_L-ENC_L_old)*(-DIST_PER_PULSE))/ENCDIST;
	POS_SIGMA = (dL_R - dL_L)/WHEEL_DIST;
	dL = (dL_R + dL_L)*0.50000;

	//移動距離deLpを算出
	if(fabs(POS_SIGMA)<0.0001){//左右の速度が等しく直進するとき
		POS_SIGMA = 0.0;
		detLp = dL_R;
	}else{                       //左右どちらかにマシンの向きが変わってるとき
		r = dL/POS_SIGMA;
		detLp = 2.0000*r*sin(POS_SIGMA*0.50000);
	}

/*    mchn_pose.vel_odom.x = detLp * cos(POS_ANG + (POS_SIGMA/2.0));
    mchn_pose.vel_odom.y = detLp * sin(POS_ANG + (POS_SIGMA/2.0));*/
	dX = detLp * cos(POS_ANG + (POS_SIGMA/2.0));//X,Yの前回からの移動量計算
	dY = detLp * sin(POS_ANG + (POS_SIGMA/2.0));
	ENC_R_old = ENC_R;//前回のエンコーダーの値を記録
	ENC_L_old = ENC_L;
	POS_ANG = (ENC_R_old - (-ENC_L_old))*DIST_PER_PULSE/WHEEL_DIST;//現在の角度を算出

	while(POS_ANG > 3.14159265){  //向いている角度を-180°~180°(-π~π)の範囲に合わせる
		POS_ANG = POS_ANG - (2*3.14159265);
	}
	while(POS_ANG < -3.14159265){
		POS_ANG = POS_ANG + (2*3.14159265);
	}

	//data->ddR = dL_R - data->dR;//構造体に値を入力
	//data->ddL = dL_L - data->dL;
	//data->dR = dL_R;
	//data->dL = dL_L;
	mchn_pose.pos_odom.x += dX;
    mchn_pose.pos_odom.y += dY;
    mchn_pose.pos_odom.theta = POS_ANG;
    mchn_pose.vel_odom.x =  dX * ROS_RATE;
    mchn_pose.vel_odom.y =  dY * ROS_RATE;
    mchn_pose.vel_odom.theta = (dL_R - (-dL_L))/WHEEL_DIST;

/*    POS_X += dX;
	POS_Y += dY;*/
	return ;
}

void init(){
    printf("WheelChair Bringup!\n");
    ros::NodeHandle n_private("~");
}


/*------------------------------------
 * send command to mbed in wheel chair
 * argument:
 *   arg_speed: forward speed   [mm/sec]
 *   arg_theta: CCW turn speed  [radian/sec]
 * ----------------------------------*/
void move(double arg_speed, double arg_theta){
    //ROS_INFO("X:%2.f   Theta:%2.f",arg_speed,arg_theta);
    double val_L = -Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST/2)*arg_theta);
    double val_R =  Dist2Pulse(arg_speed) + Dist2Pulse((WHEEL_DIST/2)*arg_theta);
    val_L = (int)Limit(val_L,(double)SPEED_MAX,(double)-SPEED_MAX);
    val_R = (int)Limit(val_R,(double)SPEED_MAX,(double)-SPEED_MAX);
    //ROS_INFO("val_L:%2.f   val_R:%2.f",val_L,val_R);

    //////////////////////////////////////////////////////
    //////////////////////for debuggind/////////////////////////
    spd_cmd_L = (int)val_L;
    spd_cmd_R = (int)val_R;
    string cmd_L = boost::lexical_cast<string>(val_L);
    string cmd_R = boost::lexical_cast<string>(val_R);

    string message = "@SS1," + cmd_L + "@SS2," + cmd_R;
//    string message = "@GP1@GP2@SS1," + cmd_L + "@SS2," + cmd_R;
    string reply;
    client_socket << message;
/*    client_socket >> reply;
    cout << "Response:" << reply << "\n";*/

/*   client_socket << "@GS1@GS2";
   client_socket >> reply;
   cout << "Response:" << reply << "\n";*/
}


void receiveCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel){
    move(cmd_vel->linear.x,cmd_vel->angular.z);
}

void receiveJoy(const sensor_msgs::Joy::ConstPtr& joy){
    ROS_INFO("Rrecieve joy");
    // move(joy->axes[1]*300100,joy->axes[3]*1/*10*/);
    joy_cmd_spd = joy->axes[1]*300;//600;
    joy_cmd_turn = joy->axes[3]*0.7;//1;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "wc_controller");
    ROS_INFO("wc_controller");
    ros::NodeHandle n;

    int Kp_,Ki_,Kd_;
    string s_Kp_,s_Ki_,s_Kd_;
    ros::NodeHandle nh_param("~");
    nh_param.param<int>("Kp",Kp_,4800);
    nh_param.param<int>("Ki",Ki_,30/*100*/);
    nh_param.param<int>("Kd",Kd_,40000);
    s_Kp_ = boost::lexical_cast<string>(Kp_);
    s_Ki_ = boost::lexical_cast<string>(Ki_);
    s_Kd_ = boost::lexical_cast<string>(Kd_);


    try{
        //ClientSocket client_socket ( "192.168.11.99", 4321 );
        string reply;
        try{
            //koyuusinndou 3Hz at Kp = 8000
            //client_socket << "@CR1@CR2@SM1,1@SM2,1@PP1,50@PP2,50@PI1,100@PI2,100@PD1,10@PD2,10";
            client_socket << "@CR1@CR2@SM1,1@SM2,1@PP1,"+s_Kp_+"@PP2,"+s_Kp_+"@PI1,"+s_Ki_+"@PI2,"+s_Ki_+"@PD1,"+s_Kd_+"@PD2,"+s_Kd_;
            client_socket >> reply;
        }
        catch ( SocketException& ) {}
        cout << "Response:" << reply << "\n";;

    }catch ( SocketException& e ){
        cout << "Exception was caught:" << e.description() << "\n";
    }

//    ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, receiveCmdVel);
    ros::Subscriber cmd_vel_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1, receiveJoy);

    ros::Time current_time, last_time;
    current_time 	= ros::Time::now();
    last_time 		= ros::Time::now();

    //ros::Rate r(10);
    // ros::Rate r(100);
    ros::Rate   r(ROS_RATE);
    while(n.ok()){
        //3.5[rot]   = 10[sec]
        //10.0[rot]  = 27.11[sec]
        //150[pulse] = 1[sec]
        //cmd = 7596
        move(joy_cmd_spd,joy_cmd_turn);
        //mchn_pose.updateOdom();

//        Odometry();
/*    ROS_INFO("ENC_L:%8ld   ENC_R:%8ld",ENC_L,ENC_R);
    ROS_INFO("X:%d   Y:%d  ANG:%lf",POS_X,POS_Y,POS_ANG);*/
	
/*        ROS_INFO("L::cmd:%6d  raw:%6d mm:%lf    R::cmd:%6d  raw:%6d mm:%lf"
            ,spd_cmd_L,1*spd_raw_L,1*spd_mm_L
            ,spd_cmd_R,1*spd_raw_R,1*spd_mm_R);*/
        ROS_INFO("");
        last_time = current_time;
        current_time = ros::Time::now();
        ros::spinOnce();
        r.sleep();
    }
    return(0);
}

