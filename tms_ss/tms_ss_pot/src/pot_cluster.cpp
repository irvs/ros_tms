//----------------------------------------------------------
// @file   : pot_cluster_single.cpp
// @author : Watanabe Yuuta
// @version: Ver0.1.4 (since 2014.05.02)
// @date   : 2016.11.12
//----------------------------------------------------------

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>

pthread_mutex_t mutex_laser = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

#include "opencv/cv.h"
#include <sensor_msgs/LaserScan.h>
#include "tms_ss_pot/define.h"
#include "tms_ss_pot/Target.h"
#include "tms_ss_pot/Laser.h"
#include <tms_msg_ss/pot_clustering.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

std::vector<float> scanData0;
std::vector<float> scanData1;

std::vector<double> lrf0_x;
std::vector<double> lrf0_y;
std::vector<double> lrf1_x;
std::vector<double> lrf1_y;

std::vector<double>::iterator it_x;
std::vector<double>::iterator it_y;

CLaser laser;
bool CallbackCalled0 = false;
bool CallbackCalled1 = false;
void LaserSetting();
void GettingBackdata();
void ClusterArrange();

void *Clustering( void *ptr )
{
    ros::Rate r(30);
    ros::Publisher *pub = (ros::Publisher *)ptr;
    laser.Init();
    LaserSetting();
    GettingBackdata();

    CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
    CvMat *m_Translate = cvCreateMat(2, 1, CV_64F);
    CvMat *Temp = cvCreateMat(2, 1, CV_64F);
    int count;
    double theta, range;

    int iteration = 0;
    while (ros::ok())
    {
        tms_msg_ss::pot_clustering cluster;
        if (laser.m_bResetBackRangeData == false)
        {
            for (int n = 0; n < laser.m_cnMaxConnect; n++)
            {
                if (laser.m_bNodeActive[n])
                {
                    pthread_mutex_lock(&mutex_laser);

                    laser.m_LRFData[n].clear();
                    if (n == 0)
                    {
                        laser.m_LRFData[n].resize(scanData0.size());
                        for (int i = 0; i < scanData0.size(); i++)
                        {
                            laser.m_LRFData[n][i] = scanData0[i];
                        }
                    }
                    if (n == 1)
                    {
                        laser.m_LRFData[n].resize(scanData1.size());
                        for (int i = 0; i < scanData1.size(); i++)
                        {
                            laser.m_LRFData[n][i] = scanData1[i];
                        }
                    }
                    pthread_mutex_unlock(&mutex_laser);

                    laser.GetDiffLRFCluster(n);

                    cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
                    cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
                    cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

                    laser.m_LRFPoints[n].clear();
                    laser.m_LRFPoints[n].resize(laser.m_DiffLRFData[n].size());

                    for ( int i = 0; i < laser.m_DiffLRFData[n].size(); i++)
                    {
                        count = laser.m_DiffLRFData[n][i].n;
                        range = laser.m_DiffLRFData[n][i].range;
                        theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

                        cvmSet(laser.m_LRFPos[n][i], 0, 0, range * cos(deg2rad(theta)));
                        cvmSet(laser.m_LRFPos[n][i], 1, 0, range * sin(deg2rad(theta)));
                        cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);    //
                        cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);  //
                        laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0) * 1000.0;
                        laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0) * 1000.0;
                    }

                    laser.m_LRFClsPoints[n].clear();
                    laser.m_LRFClsPoints[n].resize(laser.m_LRFClsData[n].size());
                    for (int i = 0; i < laser.m_LRFClsData[n].size(); i++)
                    {
                        count = laser.m_LRFClsData[n][i].n;
                        range = laser.m_LRFClsData[n][i].range;
                        theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];
                        theta = laser.m_DivAngle[1] * count + laser.m_StartAngle[1];

                        cvmSet(laser.m_LRFClsPos[n][i], 0, 0, range * cos(deg2rad(theta)));
                        cvmSet(laser.m_LRFClsPos[n][i], 1, 0, range * sin(deg2rad(theta)));
                        cvmMul(m_Rotate, laser.m_LRFClsPos[n][i], Temp);    //
                        cvmAdd(m_Translate, Temp, laser.m_LRFClsPos[n][i]);  //

                        laser.m_LRFClsPoints[n][i].x = cvmGet(laser.m_LRFClsPos[n][i], 0, 0) * 1000.0;
                        laser.m_LRFClsPoints[n][i].y = cvmGet(laser.m_LRFClsPos[n][i], 1, 0) * 1000.0;
                        std::cout << n << " " << laser.m_LRFClsPoints[n][i].x << " " << laser.m_LRFClsPoints[n][i].y << std::endl;

                        if (n == 0)
                        {
                            lrf0_x.push_back(laser.m_LRFClsPoints[0][i].x);
                            lrf0_y.push_back(laser.m_LRFClsPoints[0][i].y);
                        }
                        if (n == 1)
                        {
                            lrf1_x.push_back(laser.m_LRFClsPoints[1][i].x);
                            lrf1_y.push_back(laser.m_LRFClsPoints[1][i].y);
                        }
                    }
                }

                if (n == 1)
                {
                    ClusterArrange();
                    cluster.cls_x = lrf0_x;
                    cluster.cls_y = lrf0_y;
                    pub->publish(cluster);
                }



            }
        }
        if (!(iteration % 100)) laser.GetBackLRFDataGaussian();
        r.sleep();
        iteration ++;
    }

    cvReleaseMat(&Temp);
    cvReleaseMat(&m_Rotate);
    cvReleaseMat(&m_Translate);

}

void ClusterArrange()
{
    it_x = lrf0_x.end();
    it_y = lrf0_y.end();
    if (lrf1_x.size() > 3)
    {
        for (int i = 0; i < lrf1_x.size() ; i++)
        {
            if (lrf0_x.size() > 3)
            {
                for (it_x = lrf0_x.end(), it_y = lrf0_y.end(); it_x != lrf0_x.begin() - 1 ; --it_x, --it_y)
                {
                    double r = 10000;
                    r = sqrt(pow(*it_x - lrf1_x[i], 2) + pow(*it_y - lrf1_y[i], 2));

                    if (r < 200)
                    {
                        if (it_x == lrf0_x.end())
                        {
                            lrf0_x.insert(it_x, lrf1_x[i]);
                            lrf0_y.insert(it_y, lrf1_y[i]);
                            break;
                        }
                        else
                        {
                            lrf0_x.insert(it_x + 1, lrf1_x[i]);
                            lrf0_y.insert(it_y + 1, lrf1_y[i]);
                            break;
                        }
                    }

                    if (it_x == lrf0_x.begin())
                    {
                        lrf0_x.insert(lrf0_x.end(), lrf1_x[i]);
                        lrf0_y.insert(lrf0_y.end(), lrf1_y[i]);
                        break;
                    }
                }
            }
            else
            {
                //nigou ari itigou nasi
                for (int i = 0; i < lrf1_x.size() ; i++)
                {
                    lrf0_x.push_back(lrf1_x[i]);
                    lrf0_y.push_back(lrf1_y[i]);
                }
                break;
            }
        }
        //dainyuu
    }
}

void GettingBackdata()
{
    ros::Rate r(30);
    if (laser.m_bNodeActive[0])
    {
        while (!CallbackCalled0)r.sleep();
    }
    if (laser.m_bNodeActive[1])
    {
        while (!CallbackCalled1)r.sleep();
    }

    if (laser.m_bResetBackRangeData == true)
    {
        for (int it = 0; it < laser.m_ring; it++)
        {
            for (int n = 0; n < laser.m_cnMaxConnect; n++)
            {
                if (laser.m_bNodeActive[n])
                {
                    pthread_mutex_lock(&mutex_laser);

                    laser.m_LRFData[n].clear();
                    if (n == 0)
                    {
                        laser.m_LRFData[n].resize(scanData0.size());
                        for (int i = 0; i < scanData0.size(); i++) laser.m_LRFData[n][i] = scanData0[i];
                    }
                    if (n == 1)
                    {
                        laser.m_LRFData[n].resize(scanData1.size());
                        for (int i = 0; i < scanData1.size(); i++) laser.m_LRFData[n][i] = scanData1[i];
                    }
                    pthread_mutex_unlock(&mutex_laser);
                }
            }
            laser.GetBackLRFDataGaussian();
            r.sleep();
        }
        laser.m_bResetBackRangeData = false;
        std::cout << "Back range data is stored" << std::endl;
    }
}

void LaserSetting()
{
    //0 unit setting
    /**********************************/
    laser.m_bNodeActive[0] = true;
    laser.m_nConnectNum = 1;
    laser.GetLRFParam();
    laser.m_nStep[0] = laser.m_LRFParam[0].step;
    laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle / 2.0;
    laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
    /**********************************/

    //1 unit setting
    /**********************************/
    laser.m_bNodeActive[1] = true;
    laser.m_nConnectNum = 2;
    laser.GetLRFParam();
    laser.m_nStep[1] = laser.m_LRFParam[1].step;
    laser.m_StartAngle[1] = -laser.m_LRFParam[1].viewangle / 2.0;
    laser.m_DivAngle[1] = laser.m_LRFParam[1].divangle;
    /**********************************/

}

void LaserSensingCallback0(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    int num = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

    if ( scanData0.size() == 0 ) scanData0.resize(num);

    for (int i = 0; i < num ; i++)
    {
        if (isnan(scan->ranges[i]) == 0)
        {
            scanData0[i] = scan->ranges[i];
        }
        else
        {
            scanData0[i] = 5.6;
        }
    }
    pthread_mutex_unlock(&mutex_laser);
    CallbackCalled0 = true;
}

void LaserSensingCallback1(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    int num = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

    if ( scanData1.size() == 0 ) scanData1.resize(num);

    for (int i = 0; i < num ; i++)
    {
        if (isnan(scan->ranges[i]) == 0)
        {
            scanData1[i] = scan->ranges[i];
        }
        else
        {
            scanData1[i] = 5.6;
        }
    }
    pthread_mutex_unlock(&mutex_laser);
    CallbackCalled1 = true;
}

int main( int argc, char **argv )
{
    std::cout << "cluster_start" << std::endl;

    pthread_t thread_p;
    pthread_t thread_v;
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "pot_cluster");
    ros::NodeHandle n;
    ros::Publisher  pub  = n.advertise<tms_msg_ss::pot_clustering>("pot_clustering", 1000);
    ros::Subscriber sub0 = n.subscribe("/LaserTracker0", 1000, LaserSensingCallback0);
    ros::Subscriber sub1 = n.subscribe("/LaserTracker1", 1000, LaserSensingCallback1);

    if ( pthread_create( &thread_v, NULL, Clustering, (void *)&pub) )
    {
        printf("error creating thread.");
        abort();
    }

    spinner.spin(); // spin() will not return until the node has been shutdown

    ros::waitForShutdown();

    if ( pthread_join( thread_v, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    scanData0.clear();
    scanData1.clear();

    return 0;


}
