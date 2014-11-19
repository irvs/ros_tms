//----------------------------------------------------------
// @file   : pot_psen_manager.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.11.18
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <tms_msg_ss/pot_tracking_psen.h>
#include <tms_msg_ss/pot_tracking_psens.h>

pthread_mutex_t mutex_laser  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

#include "tms_ss_pot/define.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

signed int d_counter_r = 0;
signed int d_counter_l = 0;
int lrf_counter0_r = 0;
int lrf_counter0_l = 0;
int lrf_counter1_r = 0;
int lrf_counter1_l = 0;
int lrf_counter2_r = 0;
int lrf_counter2_l = 0;
int lrf_counter3_r = 0;
int lrf_counter3_l = 0;
int lrf_counter4_r = 0;
int lrf_counter4_l = 0;
int lrf_counter5_r = 0;
int lrf_counter5_l = 0;

void *Management( void *ptr )
{
    while (ros::ok())
    {
        // double . toutyoutenn hasamu onajimuki
        counter_r = lrf_counter0_r - lrf_counter1_r;
        counter_l = lrf_counter0_l - lrf_counter1_l;
        if (counter_r > 10 || counter_r < -10)
        {
            std::cout << "r_Mazuidesuyo! " << counter_r << std::endl;
        }
        if (counter_l > 10 || counter_l < -10)
        {
            std::cout << "l_Mazuidesuyo! " << counter_l << std::endl;
        }
        
        //triple . toutyoutenn hasamu

    }
}

void PsenCallback0(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        lrf_counter0_r = psens->pot_tracking_psen[i].righter;
        lrf_counter0_l = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen_txt0/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
    }

    pthread_mutex_unlock(&mutex_target);

}

void PsenCallback1(const tms_msg_ss::pot_tracking_psens::ConstPtr &psens)
{
    int ID;
    float output_x;
    float output_y;
    pthread_mutex_lock(&mutex_target);
    for (int i = 0; i < psens->pot_tracking_psen.size(); i++)
    {
        char str[20];
        ID = psens->pot_tracking_psen[i].id;
        output_x = psens->pot_tracking_psen[i].x;
        output_y = psens->pot_tracking_psen[i].y;
        lrf_counter1_r = psens->pot_tracking_psen[i].righter;
        lrf_counter1_l = psens->pot_tracking_psen[i].lefter;
        sprintf(str, "psen_txt1/ID%d.txt", ID);
        std::ofstream strw;
        strw.open(str, std::ofstream::out | std::ofstream::app);
    }

    pthread_mutex_unlock(&mutex_target);
}

int main( int argc, char **argv )
{
    std::cout << "---pot_psen_manager start---" << std::endl;
    pthread_t thread_p;
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "pot_psen_manager");
    ros::NodeHandle n;
    ros::Subscriber sub0 = n.subscribe("tracking_psen0" , 1000, PsenCallback0);
    ros::Subscriber sub1 = n.subscribe("tracking_psen1" , 1000, PsenCallback1);

    if ( pthread_create( &thread_p, NULL, Management, NULL) )
    {
        printf("error creating thread.");
        abort();
    }

    spinner.spin(); // spin() will not return until the node has been shutdown

    ros::waitForShutdown();

    if ( pthread_join( thread_p, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    return 0;
}