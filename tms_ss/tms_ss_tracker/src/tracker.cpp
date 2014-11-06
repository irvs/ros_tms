/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%

#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <tms_msg_ss/tracking_points.h>
#include <tms_msg_ss/tracking_grid.h>

pthread_mutex_t mutex_laser = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_target = PTHREAD_MUTEX_INITIALIZER;

//#include "opencv2/opencv.hpp"
#include "opencv/cv.h"

#include "define.h"
#include "Target.h"
#include "Laser.h"

#include "PF.h"
#include "MultipleParticleFilter.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <iostream>
#include <ctime>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

std::vector<float> scanData;
std::vector<float> scanData1;
CLaser laser;
bool CallbackCalled = false;

ros::Publisher  pub ;
ros::Subscriber sub ;
ros::Subscriber sub1;

void *Visualization( void *ptr )
{
    int   ID;
    float X;
    float Y;
    ros::Rate r(10);
    ros::Publisher *pub = (ros::Publisher *)ptr;
    int latest_id = 0;

    while (ros::ok())
    {
        tms_msg_ss::tracking_grid grid;
        tms_msg_ss::tracking_points points;
        int id = 0;
        int flag = 0;
        for (int i = 0; i < 3; i++)
        {
            id ++;
        }
        pthread_mutex_lock(&mutex_target);

        for (int i = 0; i < MAX_TRACKING_OBJECT; i++)
        {
            if (laser.m_pTarget[i] != NULL)
            {
                if (laser.m_pTarget[i]->cnt < 10)
                {
                    //cout << laser.m_pTarget[i]->cnt << endl;
                    continue;
                }
                std::cout << "laser.m_pTarget " << laser.m_pTarget[i]->cnt << std::endl;
               ID =  (laser.m_pTarget[i]->id) % 20;
               X  = -(laser.m_pTarget[i]->py) + 5200;
               Y  =  (laser.m_pTarget[i]->px) +100;
               //std::cout << "FX FY"<< FX << " "<< FY <<std::endl;
                if(0.0<X && X < 8000.0 && 0.0<Y && Y<4500.0){
		grid.id = ID;
                grid.x  = X;
                grid.y  = Y;
               
                points.tracking_grid.push_back(grid);
                }
                id ++;
          
            }
        }
        if (id - 3 > 0) std::cout << "Number of Markers " << id - 3 << std::endl;

        pthread_mutex_unlock(&mutex_target);

        latest_id = id;

        pub->publish(points);
        //std::cout << "pub"<<std::endl;
        r.sleep();
    }
    return 0;
}


void *Processing( void *ptr )
{
    CMultipleParticleFilter m_PF;
    ros::Rate r(30);
    std::vector<double> itigou_x;
    std::vector<double> itigou_y;
    std::vector<double> nigou_x;
    std::vector<double> nigou_y;

    std::vector<double>::iterator it_x;
    std::vector<double>::iterator it_y;

    laser.Init();

    /**********************************/
    laser.m_bNodeActive[0] = true;
    laser.m_bNodeActive[1] = true;
    laser.m_nConnectNum = 2;
    laser.GetLRFParam();
    laser.m_nStep[0] = laser.m_LRFParam[0].step;
    laser.m_nStep[1] = laser.m_LRFParam[1].step;
    laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle / 2.0;
    laser.m_StartAngle[1] = -laser.m_LRFParam[1].viewangle / 2.0;
    laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
    laser.m_DivAngle[1] = laser.m_LRFParam[1].divangle;
    /**********************************/

    CvMat *m_Rotate = cvCreateMat(2, 2, CV_64F);
    CvMat *m_Translate = cvCreateMat(2, 1, CV_64F);
    CvMat *Temp = cvCreateMat(2, 1, CV_64F);
    int count;
    double theta, range;

    if (laser.m_bNodeActive[0])
        while (!CallbackCalled) r.sleep();

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
                        laser.m_LRFData[n].resize(scanData.size());
                        for (int i = 0; i < scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];
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

    int iteration = 0;
    while (ros::ok())
    {
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
                        laser.m_LRFData[n].resize(scanData.size());
                        for (int i = 0; i < scanData.size(); i++)
                        {
                            laser.m_LRFData[n][i] = scanData[i];
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
			    std::cout << "itigou_push start！" << std::endl;
                            itigou_x.push_back(laser.m_LRFClsPoints[0][i].x);
                            itigou_y.push_back(laser.m_LRFClsPoints[0][i].y);
                            std::cout << "itigou_push end！" << std::endl;
                        }
                        if (n == 1)
                        {
                            std::cout << "nigou_push start！" << std::endl;
                            nigou_x.push_back(laser.m_LRFClsPoints[1][i].x);
                            nigou_y.push_back(laser.m_LRFClsPoints[1][i].y);
                            std::cout << "nigou_push end！" << std::endl;
                        }
                    }
                }

                if (n == 1)
                {
                    std::cout << "OH！" << std::endl;
                    it_x = itigou_x.end();
                    it_y = itigou_y.end();
                    if (nigou_x.size() > 3)
                    {
                        std::cout << "OK" << std::endl;
                        for (int i = 0; i < nigou_x.size() ; i++)
                        {
                            std::cout << "Ok!" << std::endl;
                            if (itigou_x.size() > 3)
                            {
                                std::cout << "OK!!" << std::endl;
                                for (it_x = itigou_x.end(), it_y = itigou_y.end(); it_x != itigou_x.begin() - 1 ; --it_x, --it_y)
                                {
                                    double r = 10000;
                                    r = sqrt(pow(*it_x - nigou_x[i], 2) + pow(*it_y - nigou_y[i], 2));

                                    if (r < 450.0)
                                    {
                                        if (it_x == itigou_x.end())
                                        {
                                            std::cout << "1-1-1 start" << std::endl;
                                            itigou_x.insert(it_x, nigou_x[i]);
                                            itigou_y.insert(it_y, nigou_y[i]);
                                            std::cout << "1-1-1 end" << std::endl;
                                            break;
                                        }
                                        else
                                        {
                                            std::cout << "1-1-2 start" << std::endl;
                                            itigou_x.insert(it_x + 1, nigou_x[i]);
                                            itigou_y.insert(it_y + 1, nigou_y[i]);
                                            std::cout << "1-1-2 end" << std::endl;
                                            break;
                                        }
                                    }

                                    if (it_x == itigou_x.begin())
                                    {
                                        std::cout << "1-2 start" << std::endl;
                                        itigou_x.insert(itigou_x.end(), nigou_x[i]);
                                        itigou_y.insert(itigou_y.end(), nigou_y[i]);
                                        std::cout << "1-2 end" << std::endl;
                                        break;
                                    }
                                }
                            }
                            else
                            {
                                //nigou ari itigou nasi
                                for (int i = 0; i < nigou_x.size() ; i++)
                                {
                                    std::cout << "1-3 start" << std::endl;
                                    itigou_x.push_back(nigou_x[i]);
                                    itigou_y.push_back(nigou_y[i]);
                                    std::cout << "1-3 end" << std::endl;
                                }
                                break;
                            }
                        }
                        //dainyuu

                        laser.m_LRFClsPoints[0].clear();
                        laser.m_LRFClsPoints[0].resize(itigou_x.size());
                        for (int i = 0; i < itigou_x.size() ; i++)
                        {
                            std::cout << "1-4-1 start" << std::endl;
                            laser.m_LRFClsPoints[0][i].x = itigou_x[i];
                            laser.m_LRFClsPoints[0][i].y = itigou_y[i];
                            std::cout << "1-4-1 end" << std::endl;
                        }
                    }
                    else
                    {
                        // nigou nasi
                        laser.m_LRFClsPoints[0].clear();
                        laser.m_LRFClsPoints[0].resize(itigou_x.size());
                        for (int i = 0; i < itigou_x.size() ; i++)
                        {
                            std::cout << "1-4-2 start" << std::endl;
                            laser.m_LRFClsPoints[0][i].x = itigou_x[i];
                            laser.m_LRFClsPoints[0][i].y = itigou_y[i];
                            std::cout << "1-4-2 end" << std::endl;
                        }
                    }

                    itigou_x.clear();
                    itigou_y.clear();
                    nigou_x.clear();
                    nigou_y.clear();
                }

            }

        }
std::cout << "m_PF start！" << std::endl;
        m_PF.update(&laser);
std::cout << "m_PF end！" << std::endl;
        ros::Time begin = ros::Time::now();
        if (m_PF.m_ParticleFilter.size() > 0) std::cout << "Time " << begin << " Number of PFs " << m_PF.m_ParticleFilter.size() << std::endl;

        if (!(iteration % 100)) laser.GetBackLRFDataGaussian();

        r.sleep();
        iteration ++;
    }

    cvReleaseMat(&Temp);
    cvReleaseMat(&m_Rotate);
    cvReleaseMat(&m_Translate);

}

void LaserSensingCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    int num = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

    if ( scanData.size() == 0 ) scanData.resize(num);

            scanData = scan->ranges;

    pthread_mutex_unlock(&mutex_laser);
    CallbackCalled = true;
}

void LaserSensingCallback1(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    pthread_mutex_lock(&mutex_laser);
    int num = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);

    if ( scanData1.size() == 0 ) scanData1.resize(num);

           scanData1 = scan->ranges;

    pthread_mutex_unlock(&mutex_laser);
    CallbackCalled = true;
}

int main( int argc, char **argv )
{
    pthread_t thread_p;
    pthread_t thread_v;
    ros::MultiThreadedSpinner spinner(4);

    ros::init(argc, argv, "tracker");
    ros::NodeHandle n;
    ros::Publisher  pub  = n.advertise<tms_msg_ss::tracking_points>("tracking_points", 10);
    ros::Subscriber sub  = n.subscribe("/urg2/most_intense" , 10, LaserSensingCallback);
    ros::Subscriber sub1 = n.subscribe("/urg1/most_intense" , 10, LaserSensingCallback1);
    if ( pthread_create( &thread_v, NULL, Visualization, (void *)&pub) )
    {
        printf("error creating thread.");
        abort();
    }

    if ( pthread_create( &thread_p, NULL, Processing, NULL) )
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
    if ( pthread_join( thread_v, NULL) )
    {
        printf("error joining thread.");
        abort();
    }

    scanData.clear();

    return 0;
}

