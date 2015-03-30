#!/usr/bin/env python
# -*- coding:utf-8 -*-

from ibs import CLoadCell
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
    pass


def main():
    print "Hello World"
    test_CLoadCell()



if __name__ == '__main__':
    main()
