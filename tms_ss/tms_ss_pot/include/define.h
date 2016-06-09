// host define.h
#pragma once

#define MAX_CONNECT 4
#define MAX_DATA_SIZE_LRF 700
#define MAX_TRACKING_OBJECT 20

//#define MAX_LRF_RANGE 55.0 // LRFの最大計測範囲
#define MIN_DIFF_DIST 0.02  // 0.02  // 前景とみなす背景からの距離（Gaussianでは使わない）
#define MIN_OBJ_PROB 0.00000000001  // 0.005  // 前景とみなす確率（Gaussianで使う）
#define N_RING 1000  // Gaussianの計算に使うデータ数
//#define	TARGET_AREA  30.0  // ターゲットの存在する範囲 レーザ中心に +-30 m
//#define N_OF_PARTICLES 100 // パーティクル数
//#define M_SIGMA 100.0      // 尤度計算用の分散
//#define POS_NOISE 0.05     // パーティクル更新時のノイズ（位置）
//#define VEL_NOISE 0.05     // パーティクル更新時のノイズ（速度）
//#define PARTICLE_AREA 0.1  // クラスタを中心にパーティクルを散布する範囲 （0.1m 四方）

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#define sqr(x) ((x) * (x))
#define ssqrs(x, y) (sqr(x) + sqr(y))
#define norm(x, y, z) sqrt(sqr(x) + sqr(y) + sqr(z))
#define deg2rad(x) ((double)x * M_PI / 180.0)

typedef struct _LRFParam
{
  double tx;
  double ty;
  double tz;
  double rx;
  double ry;
  double rz;
} LRFParam;

#define ZeroMemory(arg1, arg2) memset(arg1, 0, arg2)
#define CopyMemory(arg1, arg2, arg3) memcpy(arg1, arg2, arg3)

#include "config.h"
