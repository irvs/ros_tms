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

#ifdef WIN32
#pragma warning(disable : 4996)
#endif

#ifdef WIN32
  #include <windows.h>
  #include <string.h>
  #include <conio.h>
  #include <process.h>
#else 
  #include <ros/ros.h>
  #include <pthread.h>
  #include <std_msgs/String.h>
  #include <sensor_msgs/LaserScan.h>
  #include <sensor_msgs/MultiEchoLaserScan.h>
  #include <urg_c/urg_sensor.h>
  #include <urg_c/urg_utils.h>
  #include <visualization_msgs/Marker.h>
  #include <visualization_msgs/MarkerArray.h>
#endif

#ifdef WIN32
  HANDLE  mutex;
#else
  pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
#endif

//#include "opencv2/opencv.hpp"
#include "opencv/cv.h"

#include "define.h"
#include "Target.h"
#include "Laser.h"

#include "PF.h"
#include "MultipleParticleFilter.h"

std::vector<float> scanData;
CLaser laser;
bool CallbackCalled = false;

#ifdef WIN32
int main()
{
	CMultipleParticleFilter m_PF;

	laser.Init();
	
	scanData.resize(1080);
	for(int i=0; i<scanData.size(); i++){
		scanData[i] = 6000.0;
	}

	/**********************************/
	laser.m_bNodeActive[0] = true;
	laser.m_nConnectNum = 1;
	laser.GetLRFParam();
	laser.m_nStep[0] = laser.m_LRFParam[0].step;
	laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle/2.0;
	laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
	/**********************************/
	
	CvMat* m_Rotate = cvCreateMat(2, 2, CV_64F);
	CvMat* m_Translate = cvCreateMat(2, 1, CV_64F);
	CvMat* Temp = cvCreateMat(2, 1, CV_64F);
	int count;
	double theta, range;

	if(laser.m_bResetBackRangeData == true){
		for(int n=0;n<laser.m_cnMaxConnect;n++){
			if(laser.m_bNodeActive[n]){
				laser.m_LRFData[n].clear();
				laser.m_LRFData[n].resize(scanData.size());
				for(int i=0; i<scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];
			}
		}
		laser.GetBackLRFData();
		laser.m_bResetBackRangeData = false;
	}
#if 1
	for(int i=scanData.size()/2-10; i<scanData.size()/2+10; i++){
		scanData[i] = 4000.0;
	}
	for(int i=scanData.size()/3-10; i<scanData.size()/3+10; i++){
		scanData[i] = 4000.0;
	}
	for(int i=scanData.size()*2/3-10; i<scanData.size()*2/3+10; i++){
		scanData[i] = 4000.0;
	}
#endif
	int iteration = 0;
	while(1){

#if 1
		if (iteration > 10){
			//for (int i = scanData.size() / 4 - 10; i<scanData.size() / 4 + 10; i++){
			//	scanData[i] = 4000.0;
			//}
			for (int i = scanData.size() / 3 - 10; i<scanData.size() / 3 + 10; i++){
				scanData[i] = 6000.0;
			}
		}
#endif

		if(laser.m_bResetBackRangeData == false){
			for(int n=0;n<laser.m_cnMaxConnect;n++){
				if(laser.m_bNodeActive[n]){
					laser.m_LRFData[n].clear();
					laser.m_LRFData[n].resize(scanData.size());
					for(int i=0; i<scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];

					laser.GetDiffLRFData(n);

					cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
					cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

					laser.m_LRFPoints[n].clear();
					laser.m_LRFPoints[n].resize(laser.m_DiffLRFData[n].size());
					for( int i=0; i< laser.m_DiffLRFData[n].size(); i++){
						count = laser.m_DiffLRFData[n][i].n;
						range = laser.m_DiffLRFData[n][i].range;
						theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

						cvmSet(laser.m_LRFPos[n][i], 0, 0, range*cos(deg2rad(theta)));
						cvmSet(laser.m_LRFPos[n][i], 1, 0, range*sin(deg2rad(theta)));
						cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);		// âÒì]
						cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);	// ï¿êi
						laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0);
						laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0);

					}

					laser.m_LRFClsPoints[n].clear();
					laser.m_LRFClsPoints[n].resize(laser.m_LRFClsData[n].size());
					for (int i = 0; i< laser.m_LRFClsData[n].size(); i++){
						count = laser.m_LRFClsData[n][i].n;
						range = laser.m_LRFClsData[n][i].range;
						theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

						cvmSet(laser.m_LRFClsPos[n][i], 0, 0, range*cos(deg2rad(theta)));
						cvmSet(laser.m_LRFClsPos[n][i], 1, 0, range*sin(deg2rad(theta)));
						cvmMul(m_Rotate, laser.m_LRFClsPos[n][i], Temp);		// âÒì]
						cvmAdd(m_Translate, Temp, laser.m_LRFClsPos[n][i]);	// ï¿êi
						laser.m_LRFClsPoints[n][i].x = cvmGet(laser.m_LRFClsPos[n][i], 0, 0);
						laser.m_LRFClsPoints[n][i].y = cvmGet(laser.m_LRFClsPos[n][i], 1, 0);

					}
				}
			}
		}

		m_PF.update(&laser);

		for (vector<CPF>::iterator it = m_PF.m_ParticleFilter.begin(); it != m_PF.m_ParticleFilter.end(); ++it) {
			double t[2];
			it->GetTarget(t);
			std::cout << iteration << " " << it->GetID() << " :" << it->state[0] << " " << it->state[1] << " ( "<< t[0] << " " << t[1] << " )" << std::endl;
		}

#if 0
		for(int i=0; i<MAX_TRACKING_OBJECT; i++){
			if(laser.m_pTarget[i] != NULL) printf("(%d %lf %lf) ", laser.m_pTarget[i]->id, laser.m_pTarget[i]->px, laser.m_pTarget[i]->py); 
		}
		printf("\n");
#endif

		iteration++;

	}

	cvReleaseMat(&Temp);
	cvReleaseMat(&m_Rotate);
	cvReleaseMat(&m_Translate);
}

int main2()
{
	CParticleFilter m_PF;

	laser.Init();

	int area[2]={STAGE_X, STAGE_Y};
	m_PF.initialize(area, MAX_PARTICLE_NUM, MAX_PARTICLE_NUM_MCMC);

	scanData.resize(1080);
	for(int i=0; i<scanData.size(); i++){
		scanData[i] = 6000.0;
	}

	/**********************************/
	laser.m_bNodeActive[0] = true;
	laser.m_nConnectNum = 1;
	laser.GetLRFParam();
	laser.m_nStep[0] = laser.m_LRFParam[0].step;
	laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle/2.0;
	laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
	/**********************************/

	CvMat* m_Rotate = cvCreateMat(2, 2, CV_64F);
	CvMat* m_Translate = cvCreateMat(2, 1, CV_64F);
	CvMat* Temp = cvCreateMat(2, 1, CV_64F);
	int count;
	double theta, range;

	if(laser.m_bResetBackRangeData == true){
		for(int n=0;n<laser.m_cnMaxConnect;n++){
			if(laser.m_bNodeActive[n]){
				laser.m_LRFData[n].clear();
				laser.m_LRFData[n].resize(scanData.size());
				for(int i=0; i<scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];
			}
		}
		laser.GetBackLRFData();
		laser.m_bResetBackRangeData = false;
	}
#if 1
	for(int i=scanData.size()/2-10; i<scanData.size()/2+10; i++){
		scanData[i] = 4000.0;
	}
	for(int i=scanData.size()/3-10; i<scanData.size()/3+10; i++){
		scanData[i] = 4000.0;
	}
	for(int i=scanData.size()*2/3-10; i<scanData.size()*2/3+10; i++){
		scanData[i] = 4000.0;
	}
#endif
	while(1){
		if(laser.m_bResetBackRangeData == false){
			for(int n=0;n<laser.m_cnMaxConnect;n++){
				if(laser.m_bNodeActive[n]){
					laser.m_LRFData[n].clear();
					laser.m_LRFData[n].resize(scanData.size());
					for(int i=0; i<scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];

					laser.GetDiffLRFData(n);

					cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
					cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
					cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

					laser.m_LRFPoints[n].clear();
					laser.m_LRFPoints[n].resize(laser.m_DiffLRFData[n].size());
					for( int i=0; i< laser.m_DiffLRFData[n].size(); i++){
						count = laser.m_DiffLRFData[n][i].n;
						range = laser.m_DiffLRFData[n][i].range;
						theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

						cvmSet(laser.m_LRFPos[n][i], 0, 0, range*cos(deg2rad(theta)));
						cvmSet(laser.m_LRFPos[n][i], 1, 0, range*sin(deg2rad(theta)));
						cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);		// âÒì]
						cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);	// ï¿êi
						laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0);
						laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0);

					}
					for (int i = 0; i< laser.m_LRFClsData[n].size(); i++){
						count = laser.m_LRFClsData[n][i].n;
						range = laser.m_LRFClsData[n][i].range;
						theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

						cvmSet(laser.m_LRFClsPos[n][i], 0, 0, range*cos(deg2rad(theta)));
						cvmSet(laser.m_LRFClsPos[n][i], 1, 0, range*sin(deg2rad(theta)));
						cvmMul(m_Rotate, laser.m_LRFClsPos[n][i], Temp);		// âÒì]
						cvmAdd(m_Translate, Temp, laser.m_LRFClsPos[n][i]);	// ï¿êi
						laser.m_LRFClsPoints[n][i].x = cvmGet(laser.m_LRFClsPos[n][i], 0, 0);
						laser.m_LRFClsPoints[n][i].y = cvmGet(laser.m_LRFClsPos[n][i], 1, 0);

					}
				}
			}
		}

		m_PF.update(&laser);
#if 0
		for(int i=0; i<MAX_TRACKING_OBJECT; i++){
			if(laser.m_pTarget[i] != NULL) printf("(%d %lf %lf) ", laser.m_pTarget[i]->id, laser.m_pTarget[i]->px, laser.m_pTarget[i]->py); 
		}
		printf("\n");
#endif
	}

	cvReleaseMat(&Temp);
	cvReleaseMat(&m_Rotate);
	cvReleaseMat(&m_Translate);
}

#else

void *Visualization( void *ptr )
{
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  uint32_t shape_arrow = visualization_msgs::Marker::ARROW;
  ros::Rate r(10);
  ros::Publisher *pub = (ros::Publisher *)ptr;
  visualization_msgs::MarkerArray latest_markerArray;
  int latest_id = 0;
  float colorset[14][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{1,1,0,0},{0,1,1,0},{1,0,1,0},{1,1,1,0},
                          {0.5,0,0,0},{0,0.5,0,0},{0,0,0.5,0},{0.5,0.5,0,0},{0,0.5,0.5,0},{0.5,0,0.5,0},{0.5,0.5,0.5,0}};

  while (ros::ok())
  {
	  static int cnt=0;
std::cout << "visualization start " << cnt++ << std::endl;
    visualization_msgs::MarkerArray markerArray;
    //    markerArray.markers.clear();

    int id = 0;
  cout << "visualization marker setting" << endl;
    for(int i=0; i<3; i++){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/laser";
      marker.header.stamp = ros::Time::now();

//    marker.ns = "tracker";
      marker.id = id;
      marker.type = shape_arrow;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 2.0;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;


      switch(i){
      case 0:
         marker.pose.orientation.x = sin(90.0 * PI / 180.0 / 2.0);
         marker.pose.orientation.w = cos(90.0 * PI / 180.0 / 2.0);
         marker.color.r = 1.0f;
         break;
      case 1:
         marker.pose.orientation.z = sin(90.0 * PI / 180.0 / 2.0);
         marker.pose.orientation.w = cos(90.0 * PI / 180.0 / 2.0);
         marker.color.g = 1.0f;
         break;
      case 2:
         marker.pose.orientation.y = sin(-90.0 * PI / 180.0 / 2.0);
         marker.pose.orientation.w = cos(-90.0 * PI / 180.0 / 2.0);
         marker.color.b = 1.0f;
         break;
      }

//    marker.lifetime = ros::Duration();

      markerArray.markers.push_back(marker);
      id ++;
    }
  cout << "visualization marker save" << endl;

    for(int i=0; i<MAX_TRACKING_OBJECT; i++){
      if(laser.m_pTarget[i] != NULL){
		std::cout <<  i << std::endl;
		std::cout <<  laser.m_pTarget[i]->id << std::endl;
		

         visualization_msgs::Marker marker;
         marker.header.frame_id = "/laser";
         marker.header.stamp = ros::Time::now();

	 //         marker.ns = "tracker";
         marker.id = id;
         marker.type = shape;
         marker.action = visualization_msgs::Marker::ADD;

         marker.pose.position.x = laser.m_pTarget[i]->px * 0.001;
         marker.pose.position.y = laser.m_pTarget[i]->py * 0.001;
         marker.pose.position.z = 0.5;
         marker.pose.orientation.x = 0.0;
         marker.pose.orientation.y = 0.0;
         marker.pose.orientation.z = 0.0;
         marker.pose.orientation.w = 1.0;

         marker.scale.x = 0.1;
         marker.scale.y = 0.1;
         marker.scale.z = 1.0;

         int color = laser.m_pTarget[i]->id % 14;
         marker.color.r = colorset[color][0];
         marker.color.g = colorset[color][1];
         marker.color.b = colorset[color][2];
         marker.color.a = 1.0;

//         marker.lifetime = ros::Duration();

         markerArray.markers.push_back(marker);
         id ++;
      }
    }

    // std::cout << "Number of Markers " << markerArray.markers.size() << std::endl;
  cout << "visualization publish latest" << endl;

    if(latest_id > id) {
      for(int i=0; i<latest_markerArray.markers.size(); i++) latest_markerArray.markers[i].action = visualization_msgs::Marker::DELETE;
      pub->publish(latest_markerArray);
    }

    latest_id = id;
    latest_markerArray = markerArray;

  cout << "visualization publish" << endl;
    pub->publish(markerArray);
    //std::cout << "pub"<<std::endl;
  cout << "visualization end" << endl;

    r.sleep();
  }

  return 0;
}

void *Processing( void *ptr )
{
//  CParticleFilter m_PF;
  CMultipleParticleFilter m_PF;
  ros::Rate r(30);

  laser.Init();
  
//  int area[2]={STAGE_X, STAGE_Y};
//  m_PF.initialize(area, MAX_PARTICLE_NUM, MAX_PARTICLE_NUM_MCMC);

  /**********************************/
  laser.m_bNodeActive[0] = true;
  laser.m_nConnectNum = 1;
  laser.GetLRFParam();
  laser.m_nStep[0] = laser.m_LRFParam[0].step;
  laser.m_StartAngle[0] = -laser.m_LRFParam[0].viewangle/2.0;
  laser.m_DivAngle[0] = laser.m_LRFParam[0].divangle;
  /**********************************/
  
  CvMat* m_Rotate = cvCreateMat(2, 2, CV_64F);
  CvMat* m_Translate = cvCreateMat(2, 1, CV_64F);
  CvMat* Temp = cvCreateMat(2, 1, CV_64F);
  int count;
  double theta, range;

  if(laser.m_bNodeActive[0])
    while(!CallbackCalled) r.sleep();

  if(laser.m_bResetBackRangeData == true){
    for(int n=0;n<laser.m_cnMaxConnect;n++){
      if(laser.m_bNodeActive[n]){

        pthread_mutex_lock(&mutex);

        laser.m_LRFData[n].clear();
        laser.m_LRFData[n].resize(scanData.size());
        for (int i = 0; i<scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];

        pthread_mutex_unlock(&mutex);

      }
    }
    laser.GetBackLRFData();
    laser.m_bResetBackRangeData = false;
    std::cout << "Back range data is stored" << std::endl;
  }

  int iteration = 0;
  while (ros::ok())
  {
    if(laser.m_bResetBackRangeData == false){
      for(int n=0;n<laser.m_cnMaxConnect;n++){
        if(laser.m_bNodeActive[n]){

          pthread_mutex_lock(&mutex);
	std::cout << "processing start " << iteration << std::endl;

          laser.m_LRFData[n].clear();
          laser.m_LRFData[n].resize(scanData.size());
          for (int i = 0; i<scanData.size(); i++) laser.m_LRFData[n][i] = scanData[i];
          pthread_mutex_unlock(&mutex);

          laser.GetDiffLRFData(n);

          cvmSet(m_Rotate, 0, 0, cos(deg2rad(laser.m_LRFParam[n].rz)));
          cvmSet(m_Rotate, 0, 1, -sin(deg2rad(laser.m_LRFParam[n].rz)));
          cvmSet(m_Rotate, 1, 0, sin(deg2rad(laser.m_LRFParam[n].rz)));
          cvmSet(m_Rotate, 1, 1, cos(deg2rad(laser.m_LRFParam[n].rz)));
          cvmSet(m_Translate, 0, 0, laser.m_LRFParam[n].tx);
          cvmSet(m_Translate, 1, 0, laser.m_LRFParam[n].ty);

          laser.m_LRFPoints[n].clear();
          laser.m_LRFPoints[n].resize(laser.m_DiffLRFData[n].size());

          for( int i=0; i< laser.m_DiffLRFData[n].size(); i++){
            count = laser.m_DiffLRFData[n][i].n;
            range = laser.m_DiffLRFData[n][i].range;
            theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

            cvmSet(laser.m_LRFPos[n][i], 0, 0, range*cos(deg2rad(theta)));
            cvmSet(laser.m_LRFPos[n][i], 1, 0, range*sin(deg2rad(theta)));
            cvmMul(m_Rotate, laser.m_LRFPos[n][i], Temp);    // âÒì]
            cvmAdd(m_Translate, Temp, laser.m_LRFPos[n][i]);  // ï¿êi
            laser.m_LRFPoints[n][i].x = cvmGet(laser.m_LRFPos[n][i], 0, 0) * 1000.0;
            laser.m_LRFPoints[n][i].y = cvmGet(laser.m_LRFPos[n][i], 1, 0) * 1000.0;
          }

          // Number of clusters
          // std::cout << "Number of clusters " << laser.m_LRFClsData[n].size() << std::endl;

          laser.m_LRFClsPoints[n].clear();
          laser.m_LRFClsPoints[n].resize(laser.m_LRFClsData[n].size());
          for (int i = 0; i< laser.m_LRFClsData[n].size(); i++){
            count = laser.m_LRFClsData[n][i].n;
            range = laser.m_LRFClsData[n][i].range;
            theta = laser.m_DivAngle[0] * count + laser.m_StartAngle[0];

            cvmSet(laser.m_LRFClsPos[n][i], 0, 0, range*cos(deg2rad(theta)));
            cvmSet(laser.m_LRFClsPos[n][i], 1, 0, range*sin(deg2rad(theta)));
            cvmMul(m_Rotate, laser.m_LRFClsPos[n][i], Temp);    // âÒì]
            cvmAdd(m_Translate, Temp, laser.m_LRFClsPos[n][i]);  // ï¿êi
            laser.m_LRFClsPoints[n][i].x = cvmGet(laser.m_LRFClsPos[n][i], 0, 0) * 1000.0;
            laser.m_LRFClsPoints[n][i].y = cvmGet(laser.m_LRFClsPos[n][i], 1, 0) * 1000.0;

            //std::cout << laser.m_LRFClsPoints[n][i].x << " " << laser.m_LRFClsPoints[n][i].y << std::endl;

          }
        }
      }
    }

    m_PF.update(&laser);

    // std::cout << "Number of PFs " << m_PF.m_ParticleFilter.size() << std::endl;

#if 0
    for (vector<CPF>::iterator it = m_PF.m_ParticleFilter.begin(); it != m_PF.m_ParticleFilter.end(); ++it) {
        double t[2];
        it->GetTarget(t);
        std::cout << iteration << " " << it->GetID() << " :" << it->state[0] << " " << it->state[1] << " ( "<< t[0] << " " << t[1] << " )" << std::endl;
    }
#endif
#if 0
    for(int i=0; i<MAX_TRACKING_OBJECT; i++){
      if(laser.m_pTarget[i] != NULL) std::cout << "(" << laser.m_pTarget[i]->id << " " << laser.m_pTarget[i]->px << " " << laser.m_pTarget[i]->py << ") ";
    }
    std::cout << std::endl;
#endif

/*
    pthread_mutex_lock(&mutex);
    int n=0;
    for( std::vector<float>::iterator it = scanData.begin(); it != scanData.end(); it++, n++ )
      std::cout << *it << " ";
    std::cout << std::endl;
    std::cout << n << std::endl;
    pthread_mutex_unlock(&mutex);
*/

    std::cout << "processing end" << std::endl;
    r.sleep();
    iteration ++;
  }

  cvReleaseMat(&Temp);
  cvReleaseMat(&m_Rotate);
  cvReleaseMat(&m_Translate);

}

void LaserSensingCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  pthread_mutex_lock(&mutex);
  int num = floor((scan->angle_max-scan->angle_min)/scan->angle_increment);

  //  ROS_INFO("I heard: [%d range data]", num);

  if ( scanData.size() == 0 ) scanData.resize(num);

  scanData = scan->ranges;

  pthread_mutex_unlock(&mutex);

  CallbackCalled = true;
}

int main( int argc, char** argv )
{
  pthread_t thread_p;
  pthread_t thread_v;
  ros::MultiThreadedSpinner spinner(4);

  ros::init(argc, argv, "tracker");
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Subscriber sub = n.subscribe("urg1/most_intense", 1000, LaserSensingCallback);
 //êÿÇ¡ÇƒÇ›ÇÈ
  if ( pthread_create( &thread_v, NULL, Visualization, (void *)&pub) ) {
    printf("error creating thread.");
    abort();
  }

  if ( pthread_create( &thread_p, NULL, Processing, NULL) ) {
    printf("error creating thread.");
    abort();
  }

  spinner.spin(); // spin() will not return until the node has been shutdown

//ros::AsyncSpinner spinner(4); // Use 4 threads
//spinner.start();
  ros::waitForShutdown();

  if ( pthread_join( thread_p, NULL) ) {
    printf("error joining thread.");
    abort();
  }
  if ( pthread_join( thread_v, NULL) ) {
    printf("error joining thread.");
    abort();
  }

  scanData.clear();

  return 0;
}

#endif
