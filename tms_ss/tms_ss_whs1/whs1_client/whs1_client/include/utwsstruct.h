// Application Softwear Development Kit for Uniontool heart rate sonsor
// API header file version 1.1.0.24
// Copyright (C) 2011 UNIONTOOL CO.
//注意：文字列はすべてANSI(マルチバイト)です．

// version 0.0.1.21 RRD1DataEx関数追加
// version 1.1.0.24 1拍1送信モード対応，メモリモード加速度1秒更新モード対応
//                  struct WHS1EcgDataEx 追加

#pragma once

#include <Windows.h>

//構造体

//RRD1受信データの要素
typedef struct RRD1EcgData_
{
  //ECGデータ
  WORD ecg;
  //温度
  double temp;
  //加速度
  double acc_x, acc_y, acc_z;
} RRD1EcgData;

//RRD1受信データ
typedef struct RRD1Data_
{
  //時刻
  WORD year, month, day, hour, min, sec;
  //設定情報
  BYTE mode;
  //仮ID
  BYTE tempID;
  //送出番号
  BYTE sendedID;

  //心拍センサのモード　0:心拍波形 1:心拍間隔 2:心拍数
  BYTE ecg_mode;
  //加速度センサモード 0:移動平均 1:ピークホールド
  BYTE acc_mode;
  //ローバッテリーか　0:正常 1:ローバッテリー
  BYTE lowbattery;
  //1拍1送信かどうか. 0:1拍1送信 1:3拍1送信(通常モード)
  BYTE sampling_freq;

  //有効なデータ数
  BYTE data_count;
  //データ
  RRD1EcgData data[10];
} RRD1Data;

//RRD1受信データ(ミリ秒取得バージョン
typedef struct RRD1DataEx_
{
  //時刻
  WORD year, month, day, hour, min, sec, msec;
  //設定情報
  BYTE mode;
  //仮ID
  BYTE tempID;
  //送出番号
  BYTE sendedID;

  //心拍センサのモード　0:心拍波形 1:心拍間隔 2:心拍数
  BYTE ecg_mode;
  //加速度センサモード 0:移動平均 1:ピークホールド
  BYTE acc_mode;
  //ローバッテリーか　0:正常 1:ローバッテリー
  BYTE lowbattery;
  //1拍1送信かどうか. 0:1拍1送信 1:3拍1送信(通常モード)
  BYTE sampling_freq;

  //有効なデータ数
  BYTE data_count;
  //データ
  RRD1EcgData data[10];

  //その他の情報を格納するためのリザーブ
  BYTE reserve[10];
} RRD1DataEx;

//WHS-1の設定情報
typedef struct WHS1Config_
{
  //1 : 心拍波形 2: 心拍間隔 1000Hz 3: 心拍間隔 (加速度1秒更新) 4: 心拍数データ
  BYTE ecg_mode;
  //0 : 無線モード 1: メモリモード
  BYTE mode;
  //フラッシュ書き込み回数
  UINT flush_right_count;
  //加速度モード  0:移動平均 1: ピークホールド
  BYTE acc_mode;
  //温度オフセット
  CHAR tmp_offset;
  //CPU ID
  UINT cpu_id;
  //セットシリアルID
  UINT set_serial_id;
  //仮ID
  BYTE temp_id;
  //メモリ書き込み方式 0:上書き 1:停止
  BYTE mem_mode;
} WHS1Config;

//WHS1メモリデータの要素
typedef struct WHS1EcgData_
{
  //時刻(yearが0のとき時刻データはありません)
  WORD year, month, day, hour, mi, sec, msec;
  //ECGデータ
  WORD ecg;
  //温度
  double temp;
  //加速度
  double acc_x, acc_y, acc_z;
} WHS1EcgData;

//WHS-1のメモリデータ
typedef struct WHS1MemDataHeader_
{
  //1 : 心拍波形 2: 心拍間隔 1000Hz 3: 心拍間隔 (加速度1秒更新) 4: 心拍数データ
  BYTE ecg_mode;
   //加速度モード 0: ピークホールド 1:移動平均
  BYTE acc_mode; 
  //仮ID
  BYTE temp_id;
  //データ数
  UINT data_count;
} WHS1MemDataHeader;

//WHS1メモリデータの要素(加速度1秒更新時，プラス，マイナス加速度取得のため)
typedef struct WHS1EcgDataEx_
{
  //時刻(yearが0のとき時刻データはありません)
  WORD year, month, day, hour, mi, sec, msec;
  //ECGデータ
  WORD ecg;
  //温度
  double temp;
  //加速度
  double acc_px, acc_py, acc_pz; //プラス側最大値
  double acc_nx, acc_ny, acc_nz; //マイナス側最大値
} WHS1EcgDataEx;
