// ParticleFilter.cpp : 実装ファイル
//
#ifdef WIN32
#pragma warning(disable : 4996)
#endif

#ifdef WIN32
  #include <windows.h>
#else 
  #include <stdio.h>
  #include <stdlib.h>
  #include <math.h>
  #include <vector>
  #include <string.h>
  #include <pthread.h>
#endif

//#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "define.h"
#include "Target.h"
#include "Laser.h"
#include "PF.h"

#include "MultipleParticleFilter.h"

#ifdef WIN32
  extern HANDLE  mutex_laser;
  extern HANDLE  mutex_target;
#else
  extern pthread_mutex_t mutex_laser;
  extern pthread_mutex_t mutex_target;
#endif

CMultipleParticleFilter::CMultipleParticleFilter()
{
	m_max_ID = 100;
	m_min_distance = 1000.0;	// 1000mm以上離れて出現したときだけ、新しいPFを生成
	m_ID = 0;
}

CMultipleParticleFilter::~CMultipleParticleFilter()
{
	m_ParticleFilter.clear();
}

void CMultipleParticleFilter::update(CLaser *Laser)
{
	m_pLaser = Laser;
	double obs[2], p[2];

	for (int n = 0; n < m_pLaser->m_cnMaxConnect; n++){
		if (m_pLaser->m_bNodeActive[n]){
			std::vector<int> label(m_pLaser->m_LRFClsPoints[n].size(), -1);		// クラスタに対応するパーティクルフィルタの番号
			int pn = m_ParticleFilter.size();	// 現在のパーティクルフィルタの個数

			for (int j = 0; j < m_pLaser->m_LRFClsPoints[n].size(); j++){
				// クラスタ代表点
				obs[0] = m_pLaser->m_LRFClsPoints[n][j].x;
				obs[1] = m_pLaser->m_LRFClsPoints[n][j].y;

				// クラスタ毎に，一番近いパーティクルフィルタを探す
				double min_r = 1e10;
				int np = 0;
				for (vector<CPF>::iterator it = m_ParticleFilter.begin(); it != m_ParticleFilter.end(); ++it, ++np) {
					p[0] = it->state[0];
					p[1] = it->state[1];

					double r = sqrt(pow(p[0] - obs[0], 2) + pow(p[1] - obs[1], 2));

					if (min_r > r && r < m_min_distance){
						min_r = r;
						label[j] = np;
					}
				}

				// もし，クラスタに対応するパーティクルフィルタがなかったとき
				if (min_r >= m_min_distance){
					label[j] = pn++;
				}
			}

			std::vector<int> flg(pn, -1);		// クラスタに対応するパーティクルフィルタの番号
			for (int j = 0; j < m_pLaser->m_LRFClsPoints[n].size(); j++){
				if (label[j] < 0){
					std::cout << "Error" << std::endl;
				}
				else if (label[j] < m_ParticleFilter.size()){
					obs[0] = m_pLaser->m_LRFClsPoints[n][j].x;
					obs[1] = m_pLaser->m_LRFClsPoints[n][j].y;
					m_ParticleFilter[label[j]].SetTarget(obs);
					m_ParticleFilter[label[j]].update();
				}
				else{
					CPF pf;
					int area[2] = { STAGE_X, STAGE_Y };
					pf.initialize(area);

					obs[0] = m_pLaser->m_LRFClsPoints[n][j].x;
					obs[1] = m_pLaser->m_LRFClsPoints[n][j].y;
					pf.SetTarget(obs);
					pf.SetID(m_ID++);
					pf.update();
					m_ParticleFilter.push_back(pf);
				}
				flg[label[j]] = 1;
			}

			int np = 0;
			for (vector<CPF>::iterator it = m_ParticleFilter.begin(); it != m_ParticleFilter.end(); ++it, ++np) {
				if (flg[np] < 0)  {
					it->clear();
					it = m_ParticleFilter.erase(it);
					if (it == m_ParticleFilter.end()) break;
				}
			}

#ifndef WIN32
			pthread_mutex_lock(&mutex_target);
#endif
			for (int i = 0; i < MAX_TRACKING_OBJECT; i++){
				delete m_pLaser->m_pTarget[i];
				m_pLaser->m_pTarget[i] = NULL;
			}

			np = 0;
			for (vector<CPF>::iterator it = m_ParticleFilter.begin(); it != m_ParticleFilter.end(); ++it, ++np) {
				m_pLaser->m_pTarget[np] = new CTarget();
				m_pLaser->m_pTarget[np]->id = it->GetID();
				m_pLaser->m_pTarget[np]->SetPosi(it->state[0], it->state[1]);
			}

#ifndef WIN32
			pthread_mutex_unlock(&mutex_target);
#endif
		}
	}
}
