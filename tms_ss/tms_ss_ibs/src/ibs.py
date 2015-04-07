#!/usr/bin/env python
# -*- coding:utf-8 -*-

# ------------------------------------------------------------------------------
# @file   : ibs.py
# @brief  : Intelligent Board System
# @author : Akio Shigekane, Pyo
# @version: Ver1.1.1 (since 2012.00.00)
# @date   : 2015.2.25
# ------------------------------------------------------------------------------

'''not  @todo unite CStage class and CLoadCell class
    @todo readujust threshold of GetWeightDiff
'''

import serial
import sys
import math
import time
# include <tms_msg_db/tmsdb_data.h>
# include <tms_msg_ss/ics_object_data.h>
# include <tms_msg_db/TmsdbStamped.h>
# include <tms_msg_db/Tmsdb.h>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <cstdlib>

# ------------------------------------------------------------------------------
PORT_TR = "/dev/ttyUSB0"
PORT_LC0 = "/dev/ttyACM0"


LC_MAX_SENSOR_NUM = 4
LC_GET_WEIGHT_CNT = 2
LC_GET_WEIGHT_STABLE = 12

# define MAX_OBJECT_NUM      25      #環境内に存在する全物品数

# 仕様上の固定値
TR3_STX = 0x02
TR3_ETX = 0x03
TR3_CR = 0x0D
TR3_ACK = 0x30
TR3_NACK = 0x31
TR3_UID_SEND = 0x49

TR3_ANT1 = 0x00
TR3_ANT2 = 0x01
TR3_ANT3 = 0x02
TR3_ANT4 = 0x03
TR3_ANT5 = 0x04
TR3_ANT6 = 0x05
TR3_ANT7 = 0x06
TR3_ANT8 = 0x07

TR3_ModeCommand = 0x00
TR3_ModeAutoscan = 0x01
TR3_ModeTriger = 0x02
TR3_ModePolling = 0x03
TR3_ModeEAS = 0x24
TR3_ModeInventory = 0x50
TR3_ModeRDLOOP = 0x58

TR3_UID_SIZE = 8
TR3_TAG_SIZE = 16

# ユーザ設定値
TR3_TAG_MAX = 40  # HardwareMAX: 200
TR3_MAX_COMMAND_SIZE = 30
TR3_USED_ANT_NUM = 2
IC_STAGE_NAME_SIZE = 100
IC_TAG_MAX = 20
IC_STAGES_MAX = 2
IC_OBJECT_IN = 1
IC_OBJECT_OUT = -1
IC_OBJECT_MOVE = 2
IC_OBJECT_STAY = 0

NONE = 0
EXIST = 1


# define DEBUG_IBS 0
# if     1 == DEBUG_IBS
# define D_COUT(x) do { std.cout << x; } while (0)
# else:
# define D_COUT(x)
# endif

LC_MAX_SENSOR_NUM = 4
D_COUT = sys.stdout.write


class CLoadCell(object):

    def __init__(self):
        self.__mPreSensorsWeight = [0] * LC_MAX_SENSOR_NUM
        self.__mSensorPosX = [0.0] * LC_MAX_SENSOR_NUM
        self.__mSensorPosY = [0.0] * LC_MAX_SENSOR_NUM

    def __OpenPort(self):    # 通信関連初期化
        self.__ser = serial.Serial(port=PORT_LC0, baudrate=115200)
        D_COUT("opening port : " + PORT_LC0 + "   ")
        D_COUT("\033[1K\r")

    def __ClosePort(self):
        self.__ser.close()
        D_COUT("closing port : " + PORT_LC0)
        D_COUT("\033[1K\r")

    def Close(self):
        self.ClosePort()
        print "CLOSED: LoadCell"

    def Setup(self):    # 数値初期化関連
        self.__mSensorNum = LC_MAX_SENSOR_NUM
        print "OPENING: LoadCell(port:", PORT_LC0, ")"
        self.__OpenPort()
        print "OPENED: LoadCell(port:", PORT_LC0, ")"
        self.__ClosePort()
        print "CLOSED: LoadCell(port:", PORT_LC0, ")"
        self.ResetWeight()

    # 指定されたセンサから重さを取得する
    def GetWeight(self, sensor_id):
        self.__OpenPort()
        self.__ser.write(str(sensor_id))
        # write(fd, &signal, 1)
        buf = self.__ser.read(size=15)
        print type(buf)
        self.__ClosePort()
        return int(buf.replace("OK", "")) * 5  # 0.28は経験的な値

    def ResetWeight(self, initial=[], num=10):
        if initial:   # if not empty
            self.__mPreSensorsWeight = initial
            return
        self.__mPreSensorsWeight = [0] * self.__mSensorNum
        for i in xrange(num):
            for j in xrange(len(self.__mPreSensorsWeight)):
                self.__mPreSensorsWeight[j] = self.GetWeight(j)
        self.__mPreSensorsWeight = map(lambda x: x / num, self.__mPreSensorsWeight)

    def SetSensorPos(self, sensor_num, x_list, y_list):
        self.__mSensorNum = sensor_num
        # self.__mSensorPosX = list(x_list)
        # self.__mSensorPosY = list(y_list)
        self.__mSensorPosX = x_list
        self.__mSensorPosY = y_list

        # 重量の増減（物体の増減）があるかをチェック
    # TODO: fix arguments passed by reference(fixed)
    def GetWeightDiff(self, x, y, diffs, threshold=20):
        # 出力が安定するまで待つ
        pre = [0] * self.__mSensorNum
        # buf = [LC_GET_WEIGHT_CNT][LC_MAX_SENSOR_NUM]
        buf = [[0 for i in range(LC_MAX_SENSOR_NUM)] for j in range(LC_GET_WEIGHT_CNT)]
        for i in xrange(self.__mSensorNum):
            pre[i] = self.GetWeight(i)
            # pre.append(self.GetWeight(i))
        cnt = 0    # 繰り返し回数

        while cnt < LC_GET_WEIGHT_CNT:
            # time,sleep(0.004) #4ms程度は間隔を空ける
            time.sleep(0.02)
            weight = 0
            for i in xrange(self.__mSensorNum):
                now = self.GetWeight(i)
                weight += math.fabs(now - pre[i])
                pre[i] = now
                buf[cnt][i] = now
            if weight < LC_GET_WEIGHT_STABLE:
                cnt += 1
            else:
                cnt = 0

        # 出力
        pre = [0] * self.__mSensorNum
        for i in xrange(LC_GET_WEIGHT_CNT):
            for j in xrange(self.__mSensorNum):
                pre[j] += buf[i][j]
        pre = map(lambda x: x / LC_GET_WEIGHT_CNT, pre)
        diffs = map(lambda x: x[0] - x[1], zip(pre, self.__mPreSensorsWeight))
        x = y = 0
        weight = 0
        for i in xrange(self.__mSensorNum):
            x += self.__mSensorPosX[i] * math.fabs(diffs[i])
            y += self.__mSensorPosY[i] * math.fabs(diffs[i])
            weight += diffs[i]

        if abs(weight) < threshold:
            # return 0, x, y, diffs
            return 0, 0, 0, diffs
        else:
            self.__mPreSensorsWeight = pre
            x /= math.fabs(weight)
            y /= math.fabs(weight)
            return weight, x, y, diffs


class CTR3(object):

    def __init__(self):
        self.__mActiveAntenna = 0
        self.__mUIDs = [list() for i in xrange(TR3_USED_ANT_NUM)]
        self.__mCommand = [0] * TR3_MAX_COMMAND_SIZE

    def Setup(self):
        self.__mActiveAntenna = TR3_ANT1
        self.__mCommand[0] = TR3_STX
        self.__mCommand[1] = 0x00      # アドレス
        print "OPENING: TR3(port:", PORT_TR, ")"
        self.__OpenPort()
        print "OPENED: TR3(port:", PORT_TR, ")"
        self.__ClosePort()
        print "CLOSED: TR3(port:", PORT_TR, ")"

    def __OpenPort(self):
        self.__ser = serial.Serial(port=PORT_TR, baudrate=38400)
        D_COUT("opening port : " + PORT_TR + "   ")
        D_COUT("\033[1K\r")

    def __ClosePort(self):
        self.__ser.close()
        D_COUT("closing port : " + PORT_TR)
        D_COUT("\033[1K\r")

    def Close(self):
        self.__ClosePort()
        print "CLOSED: TR3"

    # アンテナの指定
    # def SetAntenna(self, char AN):
    def SetAntenna(self, AN):
        self.__mCommand[2] = 0x4E  # コマンド
        self.__mCommand[3] = 0x02  # データ長
        self.__mCommand[4] = 0x9C  # コマンド詳細
        self.__mCommand[5] = AN    # アンテナ指定

        self.__OpenPort()
        self.AddChecksum()
        self.__ser.write("".join(map(chr, self.__mCommand)))
        buf = [int()] * 100
        buf = map(ord, self.__ser.read(size=9))
        # print "set antenna return:", buf
        if buf[2] != TR3_ACK:
            # self.__ser.read(size = 100)
            buf = self.__ser.readline()
            # read(fd, buf, 100)
            print "TR3: SendCommandError . SetAntenna"
            return -1
        self.__mActiveAntenna = buf[5]
        self.__ClosePort()
        return buf[5]

    # アンテナの電源ON
    # TODO: I dont know this method worked correctly
    def AntennaPowerON(self):
        self.__mCommand[2] = 0x4E  # コマンド
        self.__mCommand[3] = 0x02  # データ長
        self.__mCommand[4] = 0x9E  # コマンド詳細
        self.__mCommand[5] = 0x01  # パワーON

        self.__OpenPort()
        self.AddChecksum()
        self.__ser.write("".join(map(chr, self.__mCommand)))
        buf = [chr(0)] * 100
        buf = map(ord, self.__ser.read(size=9))
        if buf[2] != TR3_ACK:
            buf = self.__ser.read(size=100)
            print "TR3: SendCommandError . AntennaPowerON"
            return False
        self.__ClosePort()
        return True

    # アンテナの電源OFF
    # TODO: I dont know this method worked correctly
    def AntennaPowerOFF(self):    # unsigned long num
        self.__mCommand[2] = 0x4E  # コマンド
        self.__mCommand[3] = 0x02  # データ長
        self.__mCommand[4] = 0x9E  # コマンド詳細
        self.__mCommand[5] = 0x00  # パワーOFF

        self.__OpenPort()
        self.AddChecksum()
        self.__ser.write("".join(map(chr, self.__mCommand)))
        buf = [chr(0)] * 100
        buf = map(ord, self.__ser.read(size=9))
        if buf[2] != TR3_ACK:
            print "TR3: SendCommandError . AntennaPowerOFF"
            return False
        self.__ClosePort()
        return True

    # 各アンテナで計測されているタグIDを全て表示
    def PrintTagUIDs(self):
        for i in xrange(TR3_USED_ANT_NUM):
            print "\n.. ANTENNA ", i + 1, " .."
            for num, j in enumerate(self.__mUIDs[i]):
                # print std.setw(3), num + 1, ". ", j
                print num+1, ". ", j

    # タグの読み取り
    def Inventory2(self):
        del self.__mUIDs[self.__mActiveAntenna][:]
        # アンテナを変更しないと既読込のUIDは返さず，新規UIDのみ返す
        self.__mCommand[2] = 0x78  # コマンド
        self.__mCommand[3] = 0x03  # データ長
        self.__mCommand[4] = 0xF0  # コマンド詳細
        self.__mCommand[5] = 0x00  # アンチコリジョン有
        self.__mCommand[6] = 0x01  # 出力指定.取得データ数＋UIDデータ

        buf = [int()] * TR3_TAG_SIZE * TR3_TAG_MAX
        self.__OpenPort()
        self.AddChecksum()
        self.__ser.write("".join(map(chr, self.__mCommand)))
        buf = map(ord, self.__ser.read(size=9))

        if buf[2] != TR3_ACK:
            print "TR3: SendCommandError . Inventory2"
            time.sleep(0.1)
            self.__ser.read(size=TR3_TAG_SIZE * TR3_TAG_MAX)
            return -1
        tag_num = buf[5]  # 読み込むタグの数

        # タグ情報の読込
        for i in xrange(tag_num):
            hexs = [chr(0)] * 17
            buf = map(ord, self.__ser.read(size=TR3_TAG_SIZE))
            hexs = "{0:0>2X}{1:0>2X}{2:0>2X}{3:0>2X}{4:0>2X}{5:0>2X}{6:0>2X}{7:0>2X}".format(
                buf[12], buf[11], buf[10], buf[9], buf[8], buf[7], buf[6], buf[5])
            # print "mAcAnt:", self.__mActiveAntenna
            # print hexs
            self.__mUIDs[self.__mActiveAntenna].append(hexs)
        self.__ClosePort()
        return tag_num

    # 通信用サブ関数
    def AddChecksum(self):
        num = self.__mCommand[3] + 5
        self.__mCommand[num - 1] = TR3_ETX
        self.__mCommand[num + 1] = TR3_CR
        self.__mCommand[num] = 0x00
        for i in xrange(num):
            self.__mCommand[num] += self.__mCommand[i]
        self.__mCommand[num] %= 256
        return num + 2

    # def GetTagDiff(self, &diffUID, char AN):
    # TODO: fix arguments passed by reference(fixed)
    def GetTagDiff(self, diffUID, AN):
        self.SetAntenna(AN)
        diffUID = str()
        preUIDs = list(self.__mUIDs[self.__mActiveAntenna])
        if self.Inventory2() == -1:
            return 0, diffUID

        # タグの増減の確認
        # IDにあってpreIDにない => 追加された物品ID
        increase = [str()]
        increase = list(set(self.__mUIDs[self.__mActiveAntenna]) - set(preUIDs))
        # print set(self.__mUIDs[self.__mActiveAntenna]), set(preUIDs)

        # preIDにあってIDにない => 取り除かれた物品ID
        decrease = [""]
        decrease = list(set(preUIDs) - set(self.__mUIDs[self.__mActiveAntenna]))

        # 増減なし
        if (len(increase) == 0) and (len(decrease) == 0):
            return 0, diffUID
        # 物品追加
        if (len(increase) == 1) and (len(decrease) == 0):
            diffUID = increase[0]
            return 1, diffUID
        # 物品除去
        if (len(increase) == 0) and (len(decrease) == 1):
            diffUID = decrease[0]
            return -1, diffUID
        # @TODO:maybe, unreachable and wrong branch serquence.
        # 複数物品の同時入出時（１個ずつ検出するようにする）
        if len(increase) >= 1:
            self.__mUIDs[self.__mActiveAntenna].sort()
            diffUID = increase[0]
            return 1, diffUID
        if len(decrease) >= 1:
            self.__mUIDs[self.__mActiveAntenna].sort()
            diffUID = decrease[0]
            return -1, diffUID
        return 0, diffUID


class CTagOBJ(object):

    def __init__(self):
        self.__mUID = ""
        self.__mWeight = 0
        self.__mDiffs = [0] * LC_MAX_SENSOR_NUM
        self.__mX = 0.0
        self.__mY = 0.0
        self.__mName = "\0"
        self.__mComment = "\0"
        self.Setup()
        pass

    def Setup(self):
        # char id[TR3_UID_SIZE * 2 + 1] = {'\0'}
        tmp = ["\0"] * (TR3_UID_SIZE * 2 + 1)
        for i in xrange(TR3_UID_SIZE * 2):
            tmp[i] = 0x00
        # self.__mUID.assign(tmp)
        self.__mUID = tmp
        return True

    def __del__(self):
        self.Close()

    def Close(self):
        pass


class CStage(object):

    def __init__(self):
        self.cLoadCell = CLoadCell()
        self.mAntenna = chr(0)
        self.mStagePos = [0] * 3
        self.mName = "\0"
        std.vector < CTagOBJ > cTagObj
        pass

    def Close(self):
        pass

    def Setup(self):
        self.cLoadCell.Setup()
        return True

    # def SetSensorPos(self, sensor_num, x[], y[]):
    # TODO: fix arguments passed by reference (fixed)
    def SetSensorPos(self, sensor_num, x, y):
        self.cLoadCell.SetSensorPos(sensor_num, x, y)

    # def SetAntenna(self, char AN):
    def SetAntenna(self, AN):
        self.mAntenna = AN


class CIntelCab(object):

    def __init__(self, stage_num=IC_STAGES_MAX):
        self.cTR3 = CTR3()
        self.cStage = [CStage] * IC_STAGES_MAX
        self.mStageNum = 0
        self.Setup(stage_num)

    def __del__(self):
        self.Close()

    def Setup(self, stage_num):
        self.__mStageNum = stage_num
        return True

    def PrintObjInfo(self):
        # TODO: fix this pointer variable (fixed)
        # CTagOBJ  *cObj
        for i in xrange(mStageNum):
            print "\n", self.cStage[i].mName
            # std.cout << "\n" << std.setw(20) << std.setfill(':') << cStage[i].mName << "....." << std.endl
            for j, cObj in enumerate(self.cStage[i].cTagObj):
                # cObj = &(cStage[i].cTagObj[j])
                print j + 1, ":  UID."
                print cObj.mUID,
                # std.cout << cObj.mUID
                print "Weight=", cObj.mWeight, " X=", cObj.mX, " Y=", cObj.mY
                # printf("  Weight=%4d  X=%4.0f Y=%4.0f", cObj.mWeight, cObj.mX, cObj.mY)
                print " <", cObj.mName, ":", cObj.mComment, ">"
                # std.cout << " <" << cObj.mName << ":" << cObj.mComment << ">" << std.endl; } }

    # TODO: fix arguments passed by reference (fixed)
    def UpdateObj(self, No, cInOut):
        cObj = cTagOBJ()
        self.__cObjIn = [cTagOBJ()] * IC_STAGES_MAX
        self.__cObjOut = [cTagOBJ()] * IC_STAGES_MAX
        # static CTagOBJ  cObj, cObjIn[IC_STAGES_MAX], cObjOut[IC_STAGES_MAX]
        self.__InOutTag = [0] * IC_STAGES_MAX
        self.__InOutLC = [0] * IC_STAGES_MAX
        # static int InOutTag[IC_STAGES_MAX], InOutLC[IC_STAGES_MAX]
        value = IC_OBJECT_STAY

        if No >= mStageNum:
            return IC_OBJECT_STAY

        # タグの増減チェック
        self.cTR3.AntennaPowerON()
        # not  @todo 理解不能なif分岐．実際はアンテナ0しか使ってない（通信の返り値て強制的に0になってる）のにこれのせいでアンテナ1を使用しようとしてる．
        if self.mStageNum == 1:
            self.cTR3.SetAntenna(self.cStage[No].mAntenna + 1)
        inout, cObj.mUID = self.cTR3.GetTagDiff(cObj.mUID, self.cStage[No].mAntenna)
        self.cTR3.AntennaPowerOFF()

        # タグ数増加
        if inout > 0:
            self.__InOutTag[No] = 1
            self.__cObjIn[No] = cObj
        # タグ数減少，出庫
        elif inout < 0:
            for i in xrange(len(self.cStage[No].cTagObj)):
                # self.cStage[No].cTagObjを更新
                if self.cStage[No].cTagObj.at(i).mUID.compare(cObj.mUID) == 0:
                    self.cStage[No].cTagObj.erase(self.cStage[No].cTagObj.begin() + i)
                    self.__InOutLC[No] = 0
                    break
            self.__InOutTag[No] = 0
            # *cInOut = cObj
            cInOut = cObj
            value = IC_OBJECT_OUT

        # ロードセルの増減チェック
        # cObj.mWeight = self.cStage[No].cLoadCell.GetWeightDiff(&cObj.mX, &cObj.mY, cObj.mDiffs)
        cObj.mWeight, cObj.mX, cObj.mY, cObj.mDiffs = self.cStage[
            No].cLoadCell.GetWeightDiff(cObj.mX, cObj.mY, cObj.mDiffs)

        if (cObj.mWeight > 0) and (self.__InOutTag[No] > 0):
            # 入庫
            cObj.mUID = cObjIn[No].mUID
            self.cStage[No].cTagObj.push_back(cObj)
            self.__InOutTag[No] = 0
            self.__InOutLC[No] = 0
            # *cInOut = cObj
            cInOut = cObj
            value = IC_OBJECT_IN
        elif (cObj.mWeight > 0) and (self.__InOutLC[No] < 0):
            # 庫内移動
            cnt = TR3_TAG_MAX
            for i in xrange(len(self.cStage[No].cTagObj)):
                if self.cStage[No].cTagObj.at(i).mUID.compare(self.__cObjOut[No].mUID) == 0:
                    cnt = i
                    break
            if cnt != TR3_TAG_MAX:
                self.cStage[No].cTagObj[cnt] = cObj
                self.cStage[No].cTagObj[cnt].mUID = self.__cObjOut[No].mUID
                self.cStage[No].cTagObj[cnt].mName = self.__cObjOut[No].mName
                self.cStage[No].cTagObj[cnt].mComment = self.__cObjOut[No].mComment
                self.__InOutLC[No] = 0
                # *cInOut = self.cStage[No].cTagObj.at(cnt)
                cInOut = self.cStage[No].cTagObj.at(cnt)
                value = IC_OBJECT_MOVE
        # タグ無し物品の入庫

        # 持ち上げ
        if cObj.mWeight < 0:
            comp = 5000
            cnt = TR3_TAG_MAX
            for i in xrange(len(self.cStage[No].cTagObj)):
                sum = 0
                for j in xrange(LC_MAX_SENSOR_NUM):
                    sum += abs(abs(self.cStage[No].cTagObj.at(i).mDiffs[j]) - abs(cObj.mDiffs[j]))
                if sum < comp:
                    comp = sum
                    cnt = i
            if cnt != TR3_TAG_MAX:
                self.__cObjOut[No] = self.cStage[No].cTagObj.at(cnt)
                self.__InOutLC[No] = -1
        return value, cInOut


def main():
    print "Hello World"
    pass


# def main(self, argc, **argv):
#     # std.map<std.string, rfidValue
#     rfidValue = {}
#     rfidValue["E00401004E17F97A"] = 7001
#     rfidValue["E00401004E180E50"] = 7002
#     rfidValue["E00401004E180E58"] = 7003
#     rfidValue["E00401004E180E60"] = 7004
#     rfidValue["E00401004E180E68"] = 7005
#     rfidValue["E00401004E180EA0"] = 7006
#     rfidValue["E00401004E180EA8"] = 7007
#     rfidValue["E00401004E181C88"] = 7008
#     rfidValue["E00401004E181C87"] = 7009
#     rfidValue["E00401004E181C7F"] = 7010
#     rfidValue["E00401004E181C77"] = 7011
#     rfidValue["E00401004E181C3F"] = 7012
#     rfidValue["E00401004E181C37"] = 7013
#     rfidValue["E00401004E180E47"] = 7014
#     rfidValue["E00401004E180E3F"] = 7015
#     rfidValue["E00401004E180E37"] = 7016
#     rfidValue["E00401004E1805BD"] = 7017
#     rfidValue["E00401004E180585"] = 7018
#     rfidValue["E00401004E18057D"] = 7019
#     rfidValue["E00401004E17EF3F"] = 7020
#     rfidValue["E00401004E17EF37"] = 7021
#     rfidValue["E00401004E17EF2F"] = 7022
#     rfidValue["E00401004E17EF27"] = 7023
#     rfidValue["E00401004E17EEEF"] = 7024
#     rfidValue["E00401004E17EEE7"] = 7025

#     cIntelCab = CIntelCab(1)
#     xpos0 = [16.0, 407.0, 16.0, 407.0]
#     ypos0 = [16.0, 16.0, 244.0, 244.0]  # colorbox
#     # float xpos0[] = {16, 407, 16, 407 }, ypos0[] = {16, 16, 244, 244 }; # colorbox
#     # CTagOBJ  cObj
#     cObj = CTagOBJ()
#     # tms_msg_db.TmsdbStamped  icsmsg
#     # tms_msg_db.Tmsdb                 tmpdata
#     icsmsg = tms_msg_db.TmsdbStamped
#     tmpdata = tms_msg_db.Tmsdb
#     # int32_t idSensor, idPlace; # shelf (ics) >> 928_foor >> 928_room
#     idSensor = idPlace = 0  # shelf (ics) >> 928_foor >> 928_room

#     ros.init(argc, argv, "ics", ros.init_options.AnonymousName)
#     rospy.init_node('ics')  # TODO: add anonymous option

#     # ros.NodeHandle n
#     # ics_pub = n.advertise<tms_msg_db.TmsdbStamped>("tms_db_data", 10)
#     ics_pub = rospy.Publisher('tms_db_data', TmsdbStamped, queue_size=10)

#     # ros.NodeHandle nh_param("~")
#     # nh_param.param<std.string>("PORT_TR", PORT_TR, "/dev/ttyUSB0")
#     # nh_param.param<std.string>("PORT_LC0", PORT_LC0, "/dev/ttyACM0")
#     # std.cout << ("sudo -S chmod a+rw " + PORT_TR + " " + PORT_LC0).c_str()
#     # system(("sudo -S chmod a+rw " + PORT_TR + " " + PORT_LC0).c_str())
#     # if not  nh_param.getParam("idSensor", idSensor):
#     #       ROS_ERROR("ros param idSensor isn't exist")
#     #     return 0;
#     # if not  nh_param.getParam("idPlace", idPlace):
#     #     ROS_ERROR("ros param idPlace isn't exist")
#     #     return 0;

#     PORT_TR = "/dev/ttyUSB0"
#     PORT_LC0 = "/dev/ttyACM0"
#     idSensor = 3000
#     idPlace = 5000

#     # iniファイルから値を読み込んで各デバイスに接続
#     # RFIDリーダ接続
#     cIntelCab.cTR3.Setup()
#     cIntelCab.cTR3.AntennaPowerOFF()
#     cIntelCab.cStage[0].SetAntenna(TR3_ANT1)
#     cIntelCab.cStage[1].SetAntenna(TR3_ANT2)

#     # ロードセル接続
#     cIntelCab.cStage[0].Setup()
#     cIntelCab.cStage[0].SetSensorPos(4, xpos0, ypos0)
#     cIntelCab.cStage[0].mStagePos[0] = 0
#     cIntelCab.cStage[0].mStagePos[1] = 0
#     cIntelCab.cStage[0].mStagePos[2] = 830

#     # 初回時の起動は多少時間がかかるためここで一回実行しておく
#     for i in xrange(cIntelCab.mStageNum):
#         # cIntelCab.UpdateObj(i, &cObj)
#         cIntelCab.UpdateObj(i, cObj)

#     # 計測開始
#     change_flag = False
#     index = 0
#     print "\nSTART"

#     while not rospy.is_shutdown():        # vector 初期化
#         # 毎回初期化し，庫内にある物品だけ値を更新して送信する
#         now = ros.Time.now() + ros.Duration(9 * 60 * 60)
#         # GMT +9
#         D_COUT(datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f"))
#         icsmsg.header.frame_id = 0  # TODO: fix this value
#         # if not  nh_param.getParam("frame_id", icsmsg.header.frame_id):
#         #     ROS_ERROR("ros param frame_id isn't exist")
#         #     return 0;
#         icsmsg.header.stamp = rospy.get_rostime() + rospy.Duration(9 * 60 * 60)

#         icsmsg.tmsdb.clear()
#         for i in xrange(MAX_OBJECT_NUM):
#             usleep(1000)
#             # 1ms
#             tmpdata.time = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
#             tmpdata.id = i + 7001
#             # 物品IDは 7001 から
#             tmpdata.x = -1.0
#             tmpdata.y = -1.0
#             tmpdata.z = -1.0
#             tmpdata.place = idPlace
#             tmpdata.sensor = idSensor
#             tmpdata.state = NONE
#             # 知的収納庫内に 0:存在しない, 1:存在する
#             icsmsg.tmsdb.push_back(tmpdata)

#         for i in xrange(cIntelCab.mStageNum):  # 増減の確認
#             # switch (cIntelCab.UpdateObj(i, &cObj))
#             state, cObj = cIntelCab.UpdateObj(i, cObj)
#             if state == IC_OBJECT_STAY:
#                 change_flag = False
#             elif state == IC_OBJECT_IN:
#                 # Beep(2500,50)
#                 std.cout << "\n\n IN : "
#                 # index = (int)cIntelCab.cStage[i].cTagObj.size() - 1
#                 index = int(cIntelCab.cStage[i].cTagObj.size() - 1)
#                 cIntelCab.cStage[i].cTagObj.at(index).mName = cObj.mName
#                 cIntelCab.cStage[i].cTagObj.at(index).mComment = cObj.mComment
#                 change_flag = True
#             elif state == IC_OBJECT_MOVE:
#                 # Beep(2500,50)
#                 print "\n\nMOVE: ",
#                 change_flag = True
#             elif state == IC_OBJECT_OUT:
#                 # ##Beep(2500,50); Sleep(50); Beep(2500,50)
#                 print "\n\n OUT: ",
#                 change_flag = True
#             else:
#                 change_flag = False

#             if change_flag:
#                 change_flag = False
#                 vi = 255
#                 for j in xrange(len(cIntelCab.cStage[i].cTagObj)):
#                     cObj = cIntelCab.cStage[i].cTagObj[j]
#                     vi = rfidValue[cObj.mUID] - 7001
#                     usleep(1000)
#                     # 1ms
#                     icsmsg.tmsdb[vi].time = datetime.datetime.now().strftime(
#                         "%Y-%m-%dT%H:%M:%S.%f")
#                     icsmsg.tmsdb[vi].id = rfidValue[cObj.mUID]
#                     icsmsg.tmsdb[vi].state = EXIST
#                     # 知的収納庫内に 0:存在しない, 1:存在する
#                     icsmsg.tmsdb[vi].x = cObj.mX
#                     icsmsg.tmsdb[vi].y = cObj.mY
#                     icsmsg.tmsdb[vi].weight = cObj.mWeight
#                     # nh_param.param<float>("z",icsmsg.tmsdb[vi].z,NULL)
#                     if not nh_param.getParam("z", icsmsg.tmsdb[vi].z):
#                         ROS_ERROR("ros param z isn't exist")
#                         return 0
#                 cIntelCab.PrintObjInfo()
#                 ics_pub.publish(icsmsg)
#     return 0

if __name__ == '__main__':
    main()

# EOF