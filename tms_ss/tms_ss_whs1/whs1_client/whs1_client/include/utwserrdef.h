// Application Softwear Development Kit for Uniontool heart rate sonsor
// Error ID definition header file version 1.0.1.21
// Copyright (C) 2011 UNIONTOOL CO.

// utwserr.h
// SHINOZAKI Ryo

#pragma once

//エラー識別子
//成功(エラーなし)
#define UTWS_NO_ERR                 0x00000000
//原因不明のエラー
#define UTWS_ERR_UNKNOWN            0xffffffff
//指定されたデバイス識別子は存在しない．
#define UTWS_ERR_UNKNOWN_DEVICE_ID  0x00000001
//指定されたデバイスはすでにオープンされています．
#define UTWS_ERR_ALREADY_OPEN       0x00000002
#define UTWS_ERR_HAVE_OPEND_ALREDY  0x00000002
//デバイスを開くことができなかった．
#define UTWS_ERR_CANNOT_OPEN        0x00000003
//デバイスが閉じられています．
#define UTWS_ERR_CLOSED             0x00000004
//デバイスのモードを変更できなかった
#define UTWS_ERR_CANNOT_CHANGE_MODE 0x00000005
//無効なハンドルが指定された
#define UTWS_ERR_INVALID_HANDLE     0x00000006
//取り出し可能なデータがありません
#define UTWS_ERR_EMPTY_DATA         0x00000007
//失敗しました
#define UTWS_ERR_FAILED             0x00000008
//タイムアウト
#define UTWS_ERR_TIMEOUT            0x00000009
//デバイスのシリアル番号を取得できませんでした
#define UTWS_ERR_CANNOT_GET_SERIAL  0x0000000A
//ファイルを開けませんでした
#define UTWS_ERR_CANNOT_OPEN_FILE   0x0000000B
//宛先アドレスが間違っています
#define UTWS_ERR_BAD_DESTINATION_ADDRESS 0x0000000C
//FTD2XX.DLLが存在しません
#define UTWS_ERR_CANNOT_LOAD_FTD2XX 0x0000000D
//取得あるいは保存可能なデータが存在しません
#define UTWS_ERR_NO_DATA            0x0000000E