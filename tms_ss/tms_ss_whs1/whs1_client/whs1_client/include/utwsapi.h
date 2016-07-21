// Application Softwear Development Kit for Uniontool heart rate sonsor
// API header file version 1.1.0.24
// Copyright (C) 2011 UNIONTOOL CO.
//注意：文字列はすべてANSI(マルチバイト)です．

//version 0.0.2.11 UTWSRRD1GetDataEx関数追加
//version 1.1.0.24 1拍1送信モード対応，メモリモード加速度1秒更新モード対応のため
//                 UTWSWHS1GetReceivedMemoryEx関数追加
//                 UTWSWHS1Version関数を追加
//                 UTWSWHS1ReadMemoryEx関数を追加

#pragma once

#include <windows.h>

#ifdef UTWSAPI_EXPORTS
#define UTWS_API __declspec(dllexport)
#else
#define UTWS_API __declspec(dllimport)
#endif

//デバイス識別子
//受信機 RRD-1
#define UTWS_RRD_1    0x00000001
//送信機 WHS-1
#define UTWS_WHS_1    0x00000002

//エラー識別子のインクルード
#include "utwserrdef.h"

//構造体のインクルード
#include "utwsstruct.h"

//////共通////////////////////////////////

//エラー情報を返す
//戻り値
//直前に発生したエラーのエラー識別子を返します．
//解説
//無効な引数ならば，エラー識別子だけを返します．
//また，引数のバッファサイズがエラー文字数よりも小さいときは，バッファサイズに切り詰められます．
UTWS_API UINT WINAPI UTWSGetErrorMessage(
  //エラーに関する説明が格納される文字列バッファへのポインタ
  /*out*/ LPSTR _err_message,
  //指定されたバッファに保持できる最大文字数．終端の NULL 文字を含む
  /*in*/ UINT _max_length
);

//デバイスを開く
//戻り値
//成功するとデバイスへのハンドルが返ります．
//失敗すると 0 が返ります．
//32-bit DLL では32bit値，64-bit DLL では64bit値であることに注意してください．
UTWS_API HANDLE WINAPI UTWSOpenDevice(
  //オープンしたいデバイスの識別子を指定してください．
  /*in*/ UINT _device_id,
  //WHS1-1をオープンするときは，何番目のデバイスをオープンするか指定して下さい．
  //１台しかつながっていないときは 0として下さい．
  //RRD-1では0として下さい．
  /*in*/ UINT _device_no
);

//デバイスをクローズする
//戻り値
//成功すると TRUE が返る．失敗すると FALSE が返る．
UTWS_API BOOL WINAPI UTWSCloseDevice(
  //クローズしようとするハンドル
  /*in*/ HANDLE _handle
);

//すべてのデバイスをクローズする
//戻り値
//なし
UTWS_API VOID WINAPI UTWSCloseAll(VOID);


//// RRD-1 //////////////////////////////////

//RRD-1でデータ受信を開始する
//成功ならば TRUE が返る
//第2引数にはイベントオブジェクトへのポインタを指定します．
//0 が指定された場合，そのオブジェクトは使用されません
//受信すると，イベントはシグナル状態になります．
//シグナル状態は，UTWSRRD1GetData関数ですべてのデータが取り出されない限り
//非シグナルにはなりません．
//第3引数にはコールバック関数へのポインタを指定します．
//コールバック関数の第一引数はユーザが_callBackArgで指定します．
//受信すると，このコールバック関数が呼び出されます．
//第2引数は，ハンドルが返ります．
//コールバック関数のtypedefを以下の通りとします
typedef VOID  (CALLBACK *UTWSRRD1CallBakFunc)(VOID*, VOID*);
UTWS_API BOOL WINAPI UTWSRRD1StartReceiving(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //イベントオブジェクトのハンドル．使用しないときは 0
  /*in*/ HANDLE _event,
  //コールバック関数へのポインタ．使用しないときは 0
  /*in*/ UTWSRRD1CallBakFunc _callBack,
  //コールバック関数を呼び出すときの第1引数．
  /*in*/ VOID* _callBackArg
);

//RRD-1でデータの受信を中止する
//成功ならば TRUE が返る.
UTWS_API BOOL CALLBACK UTWSRRD1StopReceiving(
  //デバイスのハンドル
  /*in*/ HANDLE _handle
);

//RRD-1で受信したデータが何個残っているか返す
//戻り値　失敗ならば -1 が返る．
UTWS_API UINT WINAPI UTWSRRD1DataCount(
  //デバイスのハンドル
  /*in*/ HANDLE _handle
 );

//RRD-1で受信したデータを一つ取り出す．
//DLL内では，最大1024個のデータをバッファできる．
//この関数を実行すると，取り出されたデータがバッファから取り去られる．
//成功した場合は TRUE を返す．
//データバッファに何もない状態で呼び出されると，FALSE をかえし，
//ライブラリのエラーステータスは UTWS_ERR_EMPTY_DATA に設定されます．
//このときに_dataの指すバッファに返される値は不定です．
//_data が 0の時は，何もしません．
UTWS_API BOOL WINAPI UTWSRRD1GetData(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //データを受けるとバッファ（構造体）へのポインタ
 /*out*/ RRD1Data* _data
 );

//RRD-1で受信したデータを一つ取り出す．
//UTWSRRD1GetData のミリ秒単位バージョン
UTWS_API BOOL WINAPI UTWSRRD1GetDataEx(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //データを受けるとバッファ（構造体）へのポインタ
 /*out*/ RRD1DataEx* _data
 );

//RRD-1がオープンしているかどうか調べる
//戻り値　オープンしていればTURE，していなければ FALSE を返す．
UTWS_API BOOL WINAPI UTWSRRD1IsOpen(
  //デバイスのハンドル
  /*in*/ HANDLE _handle
);

//RRD-1がオープン中にUSBコネクタから取り外されたときに呼び出される
//コールバック関数を登録する．
//成功の場合は TURE をかえす．
UTWS_API BOOL WINAPI UTWSRRD1SetCallBackDisconnect(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //コールバック関数へのポインタ．使用しないときは 0
  /*in*/ UTWSRRD1CallBakFunc _callBack,
  //コールバック関数を呼び出すときの引数
  /*in*/ VOID* _callBackArg
);  

//RRD-1の無線アドレスを取得する．
//成功すれば TRUE を返します．
//引数に文字列をして返されるので,バッファには11文字以上確保して下さい．
UTWS_API BOOL WINAPI UTWSRRD1GetLocalAddress(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //送信先アドレスが返されるバッファへのポインタ
  /*out*/ CHAR* _address
);

//RRD-1の無線アドレスを設定します．
//成功すればTRUEを返します．
//引数は0ターミネートの文字列です．
UTWS_API BOOL WINAPI UTWSRRD1SetLocalAddress(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //送信先アドレスへのポインタ
  /*in*/ const CHAR* _address
);


//// WHS-1 //////////////////////////////////
//接続されているWHS-1の台数を返します．
//戻り値 : 接続している台数返します．失敗したら -1 を返します．
//_numConnected に接続予定台数を指定して下さい．
//_timeout の間に 20msec間隔で接続台数を調べます．
//接続予定台数に達したらすぐに制御が戻ります．
//_numConnectedが0のときは，_timeoutの間デバイス探します．
//このときは_timeout後に制御が戻ります．
UTWS_API UINT WINAPI UTWSWHS1CountConnected(
  //接続予定台数．未定のときは　0
  /*in*/ UINT _numConnected,
  //タイムアウト，単位msec
  /*in*/ UINT _timeout
);

//WHS-1 の設定値を読み取る
//成功すれば TRUE を返す．
UTWS_API BOOL WINAPI UTWSWHS1ReadConfig(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //設定情報を格納する構造体へのポインタ
  /*out*/ WHS1Config* _readConfig
);

//WHS-1 の設定値を書き込む
//成功すれば TRUE を返す．
UTWS_API BOOL WINAPI UTWSWHS1WriteConfig(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //設定情報を格納する構造体へのポインタ
  /*in*/ const WHS1Config* _readConfig,
  //メモリモードでメモリフルのとき 0: 先頭から上書き，1: フルになった段階で止める
  //メモリモードに設定しないときは無視される
  /*in*/ BYTE _mem_mode,
  //以下現在時刻を設定.メモリモードに設定しないときは無視されるので 0 でかまわない．
  /*in*/ WORD _year,
  /*in*/ WORD _month,
  /*in*/ WORD _day,
  /*in*/ WORD _hour,
  /*in*/ WORD _min,
  /*in*/ WORD _sec
);

//WHS-1の無線送信先アドレスを取得します．
//成功すれば TRUE を返します．
//引数に文字列をして返されるので,バッファには11文字以上確保して下さい．
UTWS_API BOOL WINAPI UTWSWHS1GetDestinationAddress(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //送信先アドレスが返されるバッファへのポインタ
  /*out*/ CHAR* _address
);

//WHS-1の無線送信先アドレスを設定します．
//成功すればTRUEを返します．
//引数は0ターミネートの文字列です．
UTWS_API BOOL WINAPI UTWSWHS1SetDestinationAddress(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //送信先アドレスへのポインタ
  /*in*/ const CHAR* _address
);

//WHS-1 からメモリの内容をダウンロードしライブラリ内のメモリに保存します．
//戻り値: 読み込んだデータ数を返します．エラーのときの戻り値は-1です．
UTWS_API UINT WINAPI UTWSWHS1ReadMemory(
  //デバイスのハンドル
  /*in*/ HANDLE _handle
);

//WHS-1 からメモリの内容をダウンロードしライブラリ内のメモリに保存します．
//戻り値: エラーのときの戻り値はFALSEです．
UTWS_API BOOL WINAPI UTWSWHS1ReadMemoryEx(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //読み出したデータのヘッダ部
  /*out*/ WHS1MemDataHeader* _header
);

//WHS-1 からメモリの内容をダウンロードしライブラリ内のメモリに保存したデータを
//を取り出す．
//戻り値 : 失敗のときfalse
UTWS_API BOOL WINAPI UTWSWHS1GetReceivedMemory(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //読み出したデータのヘッダ部
  /*out*/ WHS1MemDataHeader* _header,
  //読み出したデータのデータ部，UTWSWHS1ReadMemory関数で得られた
  //データ数だけの配列をあらかじめ用意して下さい．
  //0ならばデータを読み込みません．
  /*out*/ WHS1EcgData* _data,
  //用意した配列WHS1EcgData[]のサイズ
  /*in*/ UINT _size,
  //TRUEのとき1レコード当たりの時計を調整します
  /*in*/ BOOL _time_adjust
);
UTWS_API BOOL WINAPI UTWSWHS1GetReadedMemory(
  HANDLE _handle, WHS1MemDataHeader* _header, WHS1EcgData* _data, UINT _size, BOOL _time_adjust
);

//WHS-1 からメモリの内容をダウンロードしライブラリ内のメモリに保存したデータを
//ファイルに書き出す.
//戻り値 : 失敗のときFALSE
UTWS_API BOOL WINAPI UTWSWHS1SaveReceivedMemory(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //パス名
  /*in*/ LPCSTR _path,
  //TRUEのとき1レコード当たりの時計を調整します
  /*in*/ BOOL _time_adjust
);
UTWS_API BOOL WINAPI UTWSWHS1SaveReadedMemory(
  HANDLE _handle, LPCSTR _path, BOOL _time_adjust
);

//WHS-1 からメモリの内容をダウンロードしライブラリ内のメモリに保存したデータを
//を取り出す．(加速度1秒更新モード時に，＋加速度とー加速度を独立して取得できる）
//戻り値 : 失敗のときfalse
UTWS_API BOOL WINAPI UTWSWHS1GetReceivedMemoryEx(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //読み出したデータのヘッダ部
  /*out*/ WHS1MemDataHeader* _header,
  //読み出したデータのデータ部，UTWSWHS1ReadMemory関数で得られた
  //データ数だけの配列をあらかじめ用意して下さい．
  //0ならばデータを読み込みません．
  /*out*/ WHS1EcgDataEx* _data,
  //用意した配列WHS1EcgData[]のサイズ
  /*in*/ UINT _size,
  //TRUEのとき1レコード当たりの時計を調整します
  /*in*/ BOOL _time_adjust
);

//WHS-1のバージョンを返す．
UTWS_API BOOL WINAPI UTWSWHS1Version(
  //デバイスのハンドル
  /*in*/ HANDLE _handle,
  //バージョン
  /*out*/ UINT* _ver
);

