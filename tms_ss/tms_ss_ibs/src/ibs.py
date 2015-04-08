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
import datetime
import rospy

from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.msg import Tmsdb
# include <tms_msg_db/tmsdb_data.h>
# include <tms_msg_ss/ics_object_data.h>
# include <tms_msg_db/TmsdbStamped.h>
# include <tms_msg_db/Tmsdb.h>
# include <boost/date_time/posix_time/posix_time.hpp>
# include <cstdlib>

# ------------------------------------------------------------------------------

LC_MAX_SENSOR_NUM = 4
LC_GET_WEIGHT_CNT = 2
LC_GET_WEIGHT_STABLE = 12

MAX_OBJECT_NUM =     25      #環境内に存在する全物品数

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

LC_MAX_SENSOR_NUM = 4
D_COUT = sys.stdout.write


class CLoadCell(object):

    def __init__(self, port):
        self.__mPreSensorsWeight = [0] * LC_MAX_SENSOR_NUM
        self.__mSensorPosX = [0.0] * LC_MAX_SENSOR_NUM
        self.__mSensorPosY = [0.0] * LC_MAX_SENSOR_NUM
        self.__mSensorNum = LC_MAX_SENSOR_NUM
        self.__ser = serial.Serial(baudrate=115200)
        self.__ser.port = port
        print "OPENING: LoadCell(port:", port, ")"
        self.__OpenPort()
        print "OPENED: LoadCell(port:", port, ")"
        self.__ClosePort()
        print "CLOSED: LoadCell(port:", port, ")"
        self.__ResetWeight()

    def __OpenPort(self):    # 通信関連初期化
        D_COUT("LoadCell: opening port...")
        self.__ser.open()
        D_COUT("\033[1K\r")

    def __ClosePort(self):
        D_COUT("LoadCell: closing port...")
        self.__ser.close()
        D_COUT("\033[1K\r")

    # 指定されたセンサから重さを取得する
    def GetWeight(self, sensor_id, repeat=1):
        self.__OpenPort()
        buf = []
        for _ in xrange(repeat):
            self.__ser.flushInput
            self.__ser.write(str(sensor_id))
            tmp = self.__ser.readline()
            # print tmp
            buf.append(int(tmp.replace("O", "").replace("K", "").replace('"', ""))*5)
        self.__ClosePort()
        return reduce(lambda x, y: x+y, buf)/len(buf)

    def __ResetWeight(self, initial=[], num=10):
        if initial:   # if not empty
            self.__mPreSensorsWeight = initial
            return
        self.__mPreSensorsWeight = [0] * self.__mSensorNum
        for i in xrange(num):
            for j in xrange(len(self.__mPreSensorsWeight)):
                self.__mPreSensorsWeight[j] += self.GetWeight(j)
        self.__mPreSensorsWeight = map(lambda x: x / num, self.__mPreSensorsWeight)

    def SetSensorPos(self, sensor_num, x_list, y_list):
        self.__mSensorNum = sensor_num
        self.__mSensorPosX = tuple(x_list)
        self.__mSensorPosY = tuple(y_list)

        # 重量の増減（物体の増減）があるかをチェック
    def GetWeightDiff(self, threshold=20):
        # 出力が安定するまで待つ
        pre = [0] * self.__mSensorNum
        buf = [[0 for i in range(LC_MAX_SENSOR_NUM)] for j in range(LC_GET_WEIGHT_CNT)]
        for i in xrange(self.__mSensorNum):
            pre[i] = self.GetWeight(i)
        cnt = 0    # 繰り返し回数

        while cnt < LC_GET_WEIGHT_CNT:
            # time,sleep(0.004) #4ms程度は間隔を空ける
            time.sleep(0.02)
            weight = 0
            for i in xrange(self.__mSensorNum):
                now = self.GetWeight(i, repeat=1)
                weight += math.fabs(now - pre[i])
                pre[i] = now
                buf[cnt][i] = now
            # print weight, pre
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
        diffs = map(lambda x, y: x-y, pre, self.__mPreSensorsWeight)
        x = y = 0
        weight = 0
        for i in xrange(self.__mSensorNum):
            x += self.__mSensorPosX[i] * math.fabs(diffs[i])
            y += self.__mSensorPosY[i] * math.fabs(diffs[i])
            weight += diffs[i]

        if abs(weight) < threshold:
            return 0, 0, 0, diffs
        else:
            self.__mPreSensorsWeight = pre
            x /= math.fabs(weight)
            y /= math.fabs(weight)
            return weight, x, y, diffs


class CTR3(object):

    def __init__(self, port, AnttenaNum):
        self.__mActiveAntenna = AnttenaNum
        self.__mUIDs = [list() for i in xrange(TR3_USED_ANT_NUM)]
        self.__mCommand = [0] * TR3_MAX_COMMAND_SIZE
        self.__mCommand[0] = TR3_STX
        self.__mCommand[1] = 0x00      # アドレス
        self.__ser = serial.Serial(baudrate=38400)
        self.__ser.port = port
        print "OPENING: TR3(port:", port, ")"
        self.__OpenPort()
        print "OPENED: TR3(port:", port, ")"
        self.__ClosePort()
        print "CLOSED: TR3(port:", port, ")"
        self.__SetAntenna(self.__mActiveAntenna)

    def __OpenPort(self):
        D_COUT("TagReader: opening port...")
        self.__ser.open()
        D_COUT("\033[1K\r")

    def __ClosePort(self):
        D_COUT("TagReader: closing port...")
        self.__ser.close()
        D_COUT("\033[1K\r")

    # アンテナの指定
    def __SetAntenna(self, AN):
        self.__mCommand[2] = 0x4E  # コマンド
        self.__mCommand[3] = 0x02  # データ長
        self.__mCommand[4] = 0x9C  # コマンド詳細
        self.__mCommand[5] = AN    # アンテナ指定

        self.__OpenPort()
        self.AddChecksum()
        self.__ser.write("".join(map(chr, self.__mCommand)))
        buf = map(ord, self.__ser.read(size=9))
        if buf[2] != TR3_ACK:
            print "TR3: SendCommandError . SetAntenna"
            self.__ser.read(size=100)
            return -1
        # self.__mActiveAntenna = buf[5]  # TODO: get true active antenna number
        self.__ClosePort()
        return buf[5]

    # アンテナの電源ON
    # TODO: I dont know this method worked correctly
    def __AntennaPowerON(self):
        self.__SetAntenna(self.__mActiveAntenna)
        self.__mCommand[2] = 0x4E  # コマンド
        self.__mCommand[3] = 0x02  # データ長
        self.__mCommand[4] = 0x9E  # コマンド詳細
        self.__mCommand[5] = 0x01  # パワーON

        self.__OpenPort()
        self.AddChecksum()
        self.__ser.write("".join(map(chr, self.__mCommand)))
        buf = map(ord, self.__ser.read(size=9))
        if buf[2] != TR3_ACK:
            buf = self.__ser.read(size=100)
            print "TR3: SendCommandError . AntennaPowerON"
            return False
        self.__ClosePort()
        return True

    # アンテナの電源OFF
    # TODO: I dont know this method worked correctly
    def __AntennaPowerOFF(self):    # unsigned long num
        self.__SetAntenna(self.__mActiveAntenna+1)  # TODO: fix correct serquence

        # self.__mCommand[2] = 0x4E  # コマンド
        # self.__mCommand[3] = 0x02  # データ長
        # self.__mCommand[4] = 0x9E  # コマンド詳細
        # self.__mCommand[5] = 0x00  # パワーOFF

        # self.__OpenPort()
        # self.AddChecksum()
        # self.__ser.write("".join(map(chr, self.__mCommand)))
        # buf = [chr(0)] * 100
        # buf = map(ord, self.__ser.read(size=9))
        # if buf[2] != TR3_ACK:
        #     print "TR3: SendCommandError . AntennaPowerOFF"
        #     return False
        # self.__ClosePort()
        # return True

    # 各アンテナで計測されているタグIDを全て表示
    def PrintTagUIDs(self):
        for i in xrange(TR3_USED_ANT_NUM):
            print "\n.. ANTENNA ", i + 1, " .."
            for num, j in enumerate(self.__mUIDs[i]):
                print "{0:>3}.{1}".format(num+1, j)

    # タグの読み取り
    def Inventory2(self):
        del self.__mUIDs[self.__mActiveAntenna][:]
        # アンテナを変更しないと既読込のUIDは返さず，新規UIDのみ返す
        self.__mCommand[2] = 0x78  # コマンド
        self.__mCommand[3] = 0x03  # データ長
        self.__mCommand[4] = 0xF0  # コマンド詳細
        self.__mCommand[5] = 0x00  # アンチコリジョン有
        self.__mCommand[6] = 0x01  # 出力指定.取得データ数＋UIDデータ

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

    # TODO: fix arguments passed by reference(fixed)
    def GetTagDiff(self, diffUID, AN):
        diffUID = str()
        # print self.__mUIDs, self.__mActiveAntenna
        preUIDs = list(self.__mUIDs[self.__mActiveAntenna])
        self.__AntennaPowerON()
        if self.Inventory2() == -1:
            self.__AntennaPowerOFF()
            return 0, diffUID
        self.__AntennaPowerOFF()

        # IDにあってpreIDにない => 追加された物品ID
        # type: [str]
        increase = list(set(self.__mUIDs[self.__mActiveAntenna]) - set(preUIDs))
        # print set(self.__mUIDs[self.__mActiveAntenna]), set(preUIDs)

        # preIDにあってIDにない => 取り除かれた物品ID
        # type: [str]
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
        self.mUID = ""
        self.mWeight = 0
        self.mDiffs = [0] * LC_MAX_SENSOR_NUM
        self.mX = 0.0
        self.mY = 0.0
        self.mName = ""
        self.mComment = ""
        self.mUID = ""


class CIntelCab(object):

    def __init__(self, lc_port="/dev/ttyACM0", lc_xpos=(0.0, 0.0, 0.0, 0.0), lc_ypos=(0.0, 0.0, 0.0, 0.0),
                 tr_port="/dev/ttyUSB0", tr_antenna=TR3_ANT1):
        self.cLoadCell = CLoadCell(lc_port)
        self.cLoadCell.SetSensorPos(len(lc_xpos), lc_xpos, lc_ypos)
        self.cTR3 = CTR3(tr_port, tr_antenna)
        self.mName = "\0"
        self.TagObjList = list()

    def PrintObjInfo(self):
        print "\n{0::>20}::::::::::".format(self.mName)
        for index, cObj in enumerate(self.TagObjList):
            print "{0:>3}:  UID->".format(index+1),
            print cObj.mUID,
            print "  Weight={0:>4}  X={1:.0f} Y={2:.0f}".format(cObj.mWeight, cObj.mX, cObj.mY),
            print "<{0}:{1}>".format(cObj.mName, cObj.mComment)

    # TODO: fix arguments passed by reference (fixed)
    def UpdateObj(self):
        # init static variables
        if not hasattr(self, "_CIntelCab__cObjIn"):
            self.__cObjIn = CTagOBJ()
        if not hasattr(self, "_CIntelCab__cObjOut"):
            self.__cObjOut = CTagOBJ()
        if not hasattr(self, "_CIntelCab__InOutTag"):
            self.__InOutTag = 0
        if not hasattr(self, "_CIntelCab__InOutLC"):
            self.__InOutLC = 0

        cObj = CTagOBJ()
        cInOut = CTagOBJ()
        value = IC_OBJECT_STAY

        # タグの増減チェック
        (inout, cObj.mUID) = self.cTR3.GetTagDiff("", 0)
        # print "GetTagDiff: ", inout, cObj.mUID

        # タグ数増加
        if inout > 0:
            self.__InOutTag = 1
            self.__cObjIn = cObj
        # タグ数減少，出庫
        elif inout < 0:
            for i in xrange(len(self.TagObjList)):
                if self.TagObjList[i].mUID == cObj.mUID:
                    del(self.TagObjList[i])
                    self.__InOutLC = 0
                    break
            self.__InOutTag = 0
            cInOut = cObj
            value = IC_OBJECT_OUT

        # ロードセルの増減チェック
        cObj.mWeight, cObj.mX, cObj.mY, cObj.mDiffs = self.cLoadCell.GetWeightDiff()

        # print "mWeighr:{0}   InOutLC:{1}".format(cObj.mWeight, self.__InOutLC),
        if (cObj.mWeight > 0) and (self.__InOutTag > 0):
            # 入庫
            cObj.mUID = self.__cObjIn.mUID
            self.TagObjList.append(cObj)
            self.__InOutTag = 0
            self.__InOutLC = 0
            cInOut = cObj
            value = IC_OBJECT_IN
        elif (cObj.mWeight > 0) and (self.__InOutLC < 0):
            # 庫内移動
            cnt = TR3_TAG_MAX
            for i in xrange(len(self.TagObjList)):
                if self.TagObjList[i].mUID == self.__cObjOut.mUID:
                    cnt = i
                    break
            if cnt != TR3_TAG_MAX:
                self.TagObjList[cnt] = cObj
                self.TagObjList[cnt].mUID = self.__cObjOut.mUID
                self.TagObjList[cnt].mName = self.__cObjOut.mName
                self.TagObjList[cnt].mComment = self.__cObjOut.mComment
                self.__InOutLC = 0
                cInOut = self.TagObjList[cnt]
                value = IC_OBJECT_MOVE
        # タグ無し物品の入庫

        # 持ち上げ
        if cObj.mWeight < 0:
            comp = 5000
            cnt = TR3_TAG_MAX
            for i in xrange(len(self.TagObjList)):
                sum = 0
                for j in xrange(LC_MAX_SENSOR_NUM):
                    sum += abs(abs(self.TagObjList[i].mDiffs[j]) - abs(cObj.mDiffs[j]))
                if sum < comp:
                    comp = sum
                    cnt = i
            if cnt != TR3_TAG_MAX:
                self.__cObjOut = self.TagObjList[cnt]
                self.__InOutLC = -1
        return value, cInOut


def main():
    print "Hello World"
    rfidValue = {}
    rfidValue["E00401004E17F97A"] = 7001
    rfidValue["E00401004E180E50"] = 7002
    rfidValue["E00401004E180E58"] = 7003
    rfidValue["E00401004E180E60"] = 7004
    rfidValue["E00401004E180E68"] = 7005
    rfidValue["E00401004E180EA0"] = 7006
    rfidValue["E00401004E180EA8"] = 7007
    rfidValue["E00401004E181C88"] = 7008
    rfidValue["E00401004E181C87"] = 7009
    rfidValue["E00401004E181C7F"] = 7010
    rfidValue["E00401004E181C77"] = 7011
    rfidValue["E00401004E181C3F"] = 7012
    rfidValue["E00401004E181C37"] = 7013
    rfidValue["E00401004E180E47"] = 7014
    rfidValue["E00401004E180E3F"] = 7015
    rfidValue["E00401004E180E37"] = 7016
    rfidValue["E00401004E1805BD"] = 7017
    rfidValue["E00401004E180585"] = 7018
    rfidValue["E00401004E18057D"] = 7019
    rfidValue["E00401004E17EF3F"] = 7020
    rfidValue["E00401004E17EF37"] = 7021
    rfidValue["E00401004E17EF2F"] = 7022
    rfidValue["E00401004E17EF27"] = 7023
    rfidValue["E00401004E17EEEF"] = 7024
    rfidValue["E00401004E17EEE7"] = 7025

    # ##init ROS
    rospy.init_node('ibs', anonymous=True)
    db_pub = rospy.Publisher('tms_db_data', TmsdbStamped, queue_size=10)

    if not rospy.has_param('~idSensor'):
        print "ros param 'idSensor' isn't exist"
        return
    if not rospy.has_param('~idPlace'):
        print "ros param 'idPlace' isn't exist"
        return
    if not rospy.has_param('~z'):
        print "ros param 'z' isn't exist"
        return
    if not rospy.has_param('~frame_id'):
        print "ros param 'frame_id' isn't exist"
        return
    idSensor = rospy.get_param('~idSensor')
    idPlace = rospy.get_param('~idPlace')
    z = rospy.get_param('~z')
    frame_id = rospy.get_param('~frame_id')
    PORT_LC0 = rospy.get_param("~PORT_LC0", "/dev/ttyACM0")
    PORT_TR = rospy.get_param("~PORT_TR", "/dev/ttyUSB0")

    xpos0 = (16.0, 407.0, 16.0, 407.0)
    ypos0 = (16.0, 16.0, 244.0, 244.0)
    cIntelCab = CIntelCab(lc_port=PORT_LC0,
                          lc_xpos=xpos0,
                          lc_ypos=ypos0,
                          tr_port=PORT_TR,
                          tr_antenna=TR3_ANT1,
                          )
    # ロードセル接続
    # cIntelCab.cLoadCell.SetSensorPos(4, xpos0, ypos0)
    cObj = CTagOBJ()

    # 初回時の起動は多少時間がかかるためここで一回実行しておく
    cIntelCab.UpdateObj()

    # 計測開始
    change_flag = False
    index = 0
    print "\nSTART"
    r = rospy.Rate(10)
    while not rospy.is_shutdown():        # vector 初期化
        r.sleep()
        state, cObj = cIntelCab.UpdateObj()
        # print "state:", state
        if state == IC_OBJECT_STAY:
            change_flag = False
        elif state == IC_OBJECT_IN:
            # Beep(2500,50)
            print "\n\n IN : ",
            index = int(len(cIntelCab.TagObjList) - 1)
            cIntelCab.TagObjList[index].mName = cObj.mName
            cIntelCab.TagObjList[index].mComment = cObj.mComment
            change_flag = True
        elif state == IC_OBJECT_MOVE:
            # Beep(2500,50)
            print "\n\nMOVE: ",
            change_flag = True
        elif state == IC_OBJECT_OUT:
            # ##Beep(2500,50); Sleep(50); Beep(2500,50)
            print "\n\n OUT: ",
            change_flag = True
        else:
            change_flag = False

        if change_flag:
            change_flag = False
            # 毎回初期化し，庫内にある物品だけ値を更新して送信する
            msg = TmsdbStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = rospy.get_rostime()+rospy.Duration(9*60*60)
            for i in xrange(MAX_OBJECT_NUM):
                time.sleep(0.001)
                tmp_db = Tmsdb()
                tmp_db.time = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
                tmp_db.id = i+7001  # 物品IDは 7001 から
                tmp_db.x = -1.0
                tmp_db.y = -1.0
                tmp_db.z = -1.0
                tmp_db.place = idPlace
                tmp_db.sensor = idSensor
                tmp_db.state = NONE  # 知的収納庫内に 0:存在しない, 1:存在する
                msg.tmsdb.append(tmp_db)

            for j in xrange(len(cIntelCab.TagObjList)):
                cObj = cIntelCab.TagObjList[j]
                vi = rfidValue[cObj.mUID] - 7001
                time.sleep(0.001)  # 1ms
                msg.tmsdb[vi].time = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
                msg.tmsdb[vi].id = rfidValue[cObj.mUID]
                msg.tmsdb[vi].state = EXIST
                msg.tmsdb[vi].x = cObj.mX
                msg.tmsdb[vi].y = cObj.mY
                msg.tmsdb[vi].z = z
                msg.tmsdb[vi].weight = cObj.mWeight

            cIntelCab.PrintObjInfo()
            db_pub.publish(msg)
    return 0

if __name__ == '__main__':
    main()

# EOF
