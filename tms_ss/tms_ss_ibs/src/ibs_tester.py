#!/usr/bin/env python
# -*- coding:utf-8 -*-

import ibs
from ibs import CLoadCell
from ibs import CTagOBJ
from ibs import CTR3
from ibs import CIntelCab
from ibs import IC_OBJECT_STAY
from ibs import IC_OBJECT_IN
from ibs import IC_OBJECT_MOVE
from ibs import IC_OBJECT_OUT
from ibs import MAX_OBJECT_NUM
import time


xpos0 = [16.0, 407.0, 16.0, 407.0]
ypos0 = [16.0, 16.0, 244.0, 244.0]  # colorbox


def test_CLoadCell():
    lc = CLoadCell()
    # print lc._CLoadCell__mPreSensorsWeight, lc._CLoadCell__mSensorPosX, lc._CLoadCell__mSensorPosY
    lc.Setup()
    xpos0 = [16.0, 407.0, 16.0, 407.0]
    ypos0 = [16.0, 16.0, 244.0, 244.0]  # colorbox
    lc.SetSensorPos(4, xpos0, ypos0)
    print lc._CLoadCell__mPreSensorsWeight, lc._CLoadCell__mSensorPosX, lc._CLoadCell__mSensorPosY
    # tr3 = CTR3()
    # tr3.Setup()
    # tr3.SetAntenna(0)
    # tr3.AntennaPowerOFF()  # not worked correctly?
    while True:
        time.sleep(0.1)
        # tr3.GetTagDiff("", 0)
        print lc.GetWeightDiff(0, 0, [])


def test_CTagOBJ():
    obj = CTagOBJ()


def test_CTR3():
    tr3 = CTR3()
    tr3.Setup()
    tr3.SetAntenna(0)
    tr3.AntennaPowerOFF()  # not worked correctly?
    while True:
        time.sleep(0.1)
        # tr3.Inventory2()
        # print tr3._CTR3__mUIDs
        tr3.GetTagDiff("", 0)
        tr3.PrintTagUIDs()

    # while True:  # test like CIntelCab.UpdateObj serquence
    #     tr3.AntennaPowerON()
    #     # tr3.SetAntenna(cStage[No].mAntenna + 1)
    #     tr3.SetAntenna(0)
    #     # tr3.GetTagDiff(cObj.mUID, cStage[No].mAntenna)
    #     tr3.GetTagDiff(0,0)
    #     tr3.AntennaPowerOFF()


def test_CStage():
    pass


def test_CIntelCab():
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
    incab = CIntelCab(1)
    cObj = CTagOBJ()
    incab.cTR3.Setup()
    incab.cTR3.AntennaPowerOFF()
    incab.cStage[0].SetAntenna(0x00)
    incab.cStage[1].SetAntenna(0x01)

    incab.cStage[0].Setup()
    incab.cStage[0].SetSensorPos(4, xpos0, ypos0)
    incab.cStage[0].mStagePos[0] = 0
    incab.cStage[0].mStagePos[1] = 0
    incab.cStage[0].mStagePos[2] = 830

    while True:
        time.sleep(0.3)
        # reload(ibs)
        print "start"
        # print "mStageNum:", incab.mStageNum

        for i in xrange(incab.mStageNum):  # 増減の確認
            # switch (cIntelCab.UpdateObj(i, &cObj))
            state, cObj = incab.UpdateObj(i, cObj)
            # print "top stack:", state, cObj.mUID
            if state == IC_OBJECT_STAY:
                change_flag = False
            elif state == IC_OBJECT_IN:
                # Beep(2500,50)
                print "\n\n IN : ",
                # index = (int)incab.cStage[i].cTagObj.size() - 1
                index = int(len(incab.cStage[i].cTagObj) - 1)
                incab.cStage[i].cTagObj[index].mName = cObj.mName
                incab.cStage[i].cTagObj[index].mComment = cObj.mComment
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
                # vi = 255
                for j in xrange(len(incab.cStage[i].cTagObj)):
                    cObj = incab.cStage[i].cTagObj[j]
                    # vi = rfidValue[cObj.mUID] - 7001
                    # usleep(1000)
                    time.sleep(0.001)
                    # 1ms
                    # icsmsg.tmsdb[vi].time = datetime.datetime.now().strftime(
                    #     "%Y-%m-%dT%H:%M:%S.%f")
                    # icsmsg.tmsdb[vi].id = rfidValue[cObj.mUID]
                    # icsmsg.tmsdb[vi].state = EXIST
                    # # 知的収納庫内に 0:存在しない, 1:存在する
                    # icsmsg.tmsdb[vi].x = cObj.mX
                    # icsmsg.tmsdb[vi].y = cObj.mY
                    # icsmsg.tmsdb[vi].weight = cObj.mWeight
                    # nh_param.param<float>("z",icsmsg.tmsdb[vi].z,NULL)
                    # if not nh_param.getParam("z", icsmsg.tmsdb[vi].z):
                    #     ROS_ERROR("ros param z isn't exist")
                    #     return 0
                incab.PrintObjInfo()
                # ics_pub.publish(icsmsg)


def main():
    print "Hello World"
    # test_CLoadCell()
    # test_CTagOBJ()
    # test_CTR3()
    # test_CStage()
    test_CIntelCab()


if __name__ == '__main__':
    main()
