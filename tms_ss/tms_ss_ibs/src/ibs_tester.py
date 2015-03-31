#!/usr/bin/env python
# -*- coding:utf-8 -*-

from ibs import CLoadCell
from ibs import CTagOBJ
from ibs import CTR3
import time


def test_CLoadCell():
    lc = CLoadCell()
    print lc._CLoadCell__mPreSensorsWeight, lc._CLoadCell__mSensorPosX, lc._CLoadCell__mSensorPosY
    lc.Setup()
    xpos0 = [16.0, 407.0, 16.0, 407.0]
    ypos0 = [16.0, 16.0, 244.0, 244.0]  # colorbox
    lc.SetSensorPos(4, xpos0, ypos0)
    print lc._CLoadCell__mPreSensorsWeight, lc._CLoadCell__mSensorPosX, lc._CLoadCell__mSensorPosY
    while True:
        time.sleep(0.1)
        print lc.GetWeightDiff(0, 0, [])


def test_CTagOBJ():
    obj = CTagOBJ()


def test_CTR3():
    tr3 = CTR3()
    tr3.Setup()
    tr3.AntennaPowerOFF
    while True:
        time.sleep(0.1)
        tr3.SetAntenna(0)
        tr3.Inventory2()
        print tr3._CTR3__mUIDs

    while True:  # test like CIntelCab.UpdateObj serquence
        tr3.AntennaPowerON()
        # tr3.SetAntenna(cStage[No].mAntenna + 1)
        tr3.SetAntenna(0)
        # tr3.GetTagDiff(cObj.mUID, cStage[No].mAntenna)
        tr3.GetTagDiff(0,0)
        tr3.AntennaPowerOFF()


def main():
    print "Hello World"
    # test_CLoadCell()
    # test_CTagOBJ()
    test_CTR3()


if __name__ == '__main__':
    main()
