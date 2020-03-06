#!/usr/bin/env python
from __future__ import print_function
import rospy

from ma_robot import MARobot
from ma_base import MABase
from ma_beacon import MABeacon

if __name__ == '__main__':
    matype = rospy.get_param('multi_agent/type', 'robot')
    if matype == 'robot':
        ma = MARobot()
    elif matype == 'base':
        ma = MABase()
    elif matype == 'beacon':
        ma = MABeacon()
    else:
        print("No type", matype)

    try:
        ma.start()
    except rospy.ROSInterruptException:
        pass
