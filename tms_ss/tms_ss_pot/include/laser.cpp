#ifdef WIN32
#pragma warning(disable : 4996)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
//#include "opencv2/opencv.hpp"
#include "opencv/cv.h"

#include "define.h"
#include "target.h"
#include "laser.h"

CLaser::CLaser()
{
  // 隣り合う点で 500mm以上、距離の差があったら、断絶しているとする
  m_diff_dist = MIN_DIFF_DIST;
  m_obj_prob = MIN_OBJ_PROB;
  m_ring = N_RING;
  m_n_ring = 0;

  m_LRFData.resize(m_cnMaxConnect);
  m_BackLRFData.resize(m_cnMaxConnect);
  m_DiffLRFData.resize(m_cnMaxConnect);
  m_LRFPoints.resize(m_cnMaxConnect);
  m_LRFClsData.resize(m_cnMaxConnect);
  m_LRFClsPoints.resize(m_cnMaxConnect);
  m_BackLRFDataRing.resize(m_cnMaxConnect);
  m_BackLRFDataAve.resize(m_cnMaxConnect);
  m_BackLRFDataVar.resize(m_cnMaxConnect);

  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    m_LRFData[n].reserve(m_cnMaxDataSizeLRF);
    m_BackLRFData[n].reserve(m_cnMaxDataSizeLRF);
    m_DiffLRFData[n].reserve(m_cnMaxDataSizeLRF);
    m_LRFPoints[n].reserve(m_cnMaxDataSizeLRF);
    m_LRFClsData[n].reserve(m_cnMaxDataSizeLRF);
    m_LRFClsPoints[n].reserve(m_cnMaxDataSizeLRF);
    for (int i = 0; i < m_cnMaxDataSizeLRF; i++)
    {
      m_LRFPos[n][i] = cvCreateMat(2, 1, CV_64F);
      m_LRFClsPos[n][i] = cvCreateMat(2, 1, CV_64F);
    }

    std::vector< double > zero(m_cnMaxDataSizeLRF, 0);
    m_BackLRFDataRing[n].resize(m_ring);
    for (int i = 0; i < m_ring; i++)
    {
      m_BackLRFDataRing[n][i] = zero;
    }
    m_BackLRFDataAve[n] = zero;
    m_BackLRFDataVar[n] = zero;
  }

  for (int i = 0; i < m_cnMaxTrackingObject; i++)
  {
    m_pTarget[i] = NULL;
    m_Target_cnt[i] = 0;
  }
}

CLaser::~CLaser()
{
  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    for (int i = 0; i < m_cnMaxDataSizeLRF; i++)
    {
      cvReleaseMat(&m_LRFPos[n][i]);
      cvReleaseMat(&m_LRFClsPos[n][i]);
    }
  }
  for (int i = 0; i < m_cnMaxTrackingObject; i++)
  {
    if (m_pTarget[i] != NULL)
      delete m_pTarget[i];
  }
}

bool CLaser::Init()  // OnNewDocument()
{
  m_nConnectNum = 0;
  for (int n = 0; n < m_cnMaxConnect; n++)
    m_bNodeActive[n] = false;
  for (int n = 0; n < m_cnMaxConnect; n++)
    m_nStep[n] = 0;
  m_bResetBackRangeData = true;
  return 0;
}

int CLaser::GetLRFParam()
{
  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    LRFParam param;
    if (m_bNodeActive[n])
    {
      // Set Pole Positions
      switch (n)
      {
        case 0:
          param.tx = Config::is()->lrf1_pos[0];
          param.ty = Config::is()->lrf1_pos[1];
          param.tz = 0.0;
          param.rx = 0.0;
          param.ry = 0.0;
          param.rz = Config::is()->lrf1_pos[2];
          break;
        case 1:
          param.tx = Config::is()->lrf2_pos[0];
          param.ty = Config::is()->lrf2_pos[1];
          param.tz = 0.0;
          param.rx = 0.0;
          param.ry = 0.0;
          param.rz = Config::is()->lrf2_pos[2];
          break;
        case 2:
          param.tx = Config::is()->lrf3_pos[0];
          param.ty = Config::is()->lrf3_pos[1];
          param.tz = 0.0;
          param.rx = 0.0;
          param.ry = 0.0;
          param.rz = Config::is()->lrf3_pos[2];
          break;
        case 3:
          param.tx = Config::is()->lrf4_pos[0];
          param.ty = Config::is()->lrf4_pos[1];
          param.tz = 0.0;
          param.rx = 0.0;
          param.ry = 0.0;
          param.rz = Config::is()->lrf4_pos[2];
          break;
        case 4:
          param.tx = Config::is()->lrf5_pos[0];
          param.ty = Config::is()->lrf5_pos[1];
          param.tz = 0.0;
          param.rx = 0.0;
          param.ry = 0.0;
          param.rz = Config::is()->lrf5_pos[2];
          break;
      }
      CopyMemory(&m_LRFParam[n], &param, sizeof(LRFParam));
    }
    else
    {
      ZeroMemory(&m_LRFParam[n], sizeof(LRFParam));
    }
  }

  return 0;
}

int CLaser::GetBackLRFData()
{
  int backcnt = 100;  // 背景作成に何回スキャンするか

  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    if (m_bNodeActive[n])
    {
      m_BackLRFData[n] = m_LRFData[n];
    }
  }
  return 0;
}

int CLaser::GetBackLRFDataGaussian()
{
  static int count = 0;
  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    if (m_bNodeActive[n])
    {
      if (count < m_ring)
      {
        for (int i = 0; i < m_LRFData[n].size(); i++)
        {
          m_BackLRFDataAve[n][i] += m_LRFData[n][i];
          m_BackLRFDataVar[n][i] += (m_LRFData[n][i] * m_LRFData[n][i]);
          if (count == m_ring - 1)
          {
            m_BackLRFDataAve[n][i] /= (double)m_ring;
            m_BackLRFDataVar[n][i] /= (double)m_ring;
            m_BackLRFDataVar[n][i] -= (m_BackLRFDataAve[n][i] * m_BackLRFDataAve[n][i]);
          }
        }
      }
      else
      {
        for (int i = 0; i < m_LRFData[n].size(); i++)
        {
          m_BackLRFDataVar[n][i] += m_BackLRFDataAve[n][i] * m_BackLRFDataAve[n][i];
          m_BackLRFDataAve[n][i] += (-m_BackLRFDataRing[n][m_n_ring][i] + m_LRFData[n][i]) / (double)m_ring;
          m_BackLRFDataVar[n][i] += (-m_BackLRFDataRing[n][m_n_ring][i] * m_BackLRFDataRing[n][m_n_ring][i] +
                                     m_LRFData[n][i] * m_LRFData[n][i]) /
                                    (double)m_ring;
          m_BackLRFDataVar[n][i] -= m_BackLRFDataAve[n][i] * m_BackLRFDataAve[n][i];
        }
      }
      m_BackLRFDataRing[n][m_n_ring] = m_LRFData[n];
    }
  }

  if (++count > m_ring)
    count = m_ring;
  m_n_ring = (++m_n_ring) % m_ring;

  return 0;
}

int CLaser::IsFrontFData(double currentdata, double backdata, double min_dist)
{
  if (fabs(currentdata - backdata) < min_dist)
    return 0;
  else
    return 1;
}

int CLaser::IsFrontFData(double currentdata, double backdata_ave, double backdata_var, double min_prob)
{
  if (currentdata > Config::is()->max_lrf_range)
    return 0;

  double diff = currentdata - backdata_ave;
  if (backdata_var != 0.0)
  {
    double prob = exp(-diff * diff / 2.0 / backdata_var) / sqrt(2.0 * M_PI * backdata_var);
    if (prob < min_prob)
    {
      return 1;
    }
  }
  return 0;
}

int CLaser::GetDiffLRFData()
{
  double diff;
  range_t d;

  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    if (m_bNodeActive[n])
    {
      m_DiffLRFData[n].clear();
      for (int i = 0; i < m_LRFData[n].size(); i++)
      {
        diff = fabs(m_LRFData[n][i] - m_BackLRFData[n][i]);
        if (diff >= m_diff_dist)
        {
          d.n = i;
          d.range = m_LRFData[n][i];
          m_DiffLRFData[n].push_back(d);
        }
      }
    }
  }

  return 0;
}

int CLaser::GetDiffLRFDataGaussian()
{
  double diff, prob;
  range_t d;

  for (int n = 0; n < m_cnMaxConnect; n++)
  {
    if (m_bNodeActive[n])
    {
      m_DiffLRFData[n].clear();
      for (int i = 0; i < m_LRFData[n].size(); i++)
      {
        diff = m_LRFData[n][i] - m_BackLRFDataAve[n][i];
        if (m_BackLRFDataVar[n][i] != 0.0)
        {
          prob = exp(-diff * diff / m_BackLRFDataVar[n][i]) / sqrt(2.0 * M_PI * m_BackLRFDataVar[n][i]);
          if (prob < m_obj_prob)
          {
            d.n = i;
            d.range = m_LRFData[n][i];
            m_DiffLRFData[n].push_back(d);
          }
        }
      }
    }
  }

  return 0;
}

int CLaser::GetDiffLRFCluster(int n)
{
  range_t d;
  cluster_t c;
  int sflg = 0;
  int sp = 0;

  if (m_bNodeActive[n])
  {
    m_DiffLRFData[n].clear();
    m_LRFClsData[n].clear();
    for (int i = 0; i < m_LRFData[n].size(); i++)
    {
      if (IsFrontFData(m_LRFData[n][i], m_BackLRFDataAve[n][i], m_BackLRFDataVar[n][i], m_obj_prob))
      {
        d.n = i;
        d.range = m_LRFData[n][i];
        m_DiffLRFData[n].push_back(d);

        if (sflg == 0)
        {
          sflg = 1;
          sp = i;
        }
        else
        {
          sflg++;
        }
      }
      else
      {
        //if (sflg > 1)
        if (sflg >= 1)
        {
          c.n = sp + sflg / 2;
          c.range = m_LRFData[n][c.n];
          if (c.range > Config::is()->min_lrf_range)
          {
            c.length = sflg;
            m_LRFClsData[n].push_back(c);
          }
        }
        sflg = 0;
      }
    }
  }

  return 0;
}
