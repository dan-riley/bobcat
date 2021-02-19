#!/usr/bin/env python
from __future__ import print_function
import rospy

from robot import BCRobot
from base import BCBase
from beacon import BCBeacon

if __name__ == '__main__':
    bctype = rospy.get_param('bobcat/type', 'robot')
    if bctype == 'robot':
        bc = BCRobot()
    elif bctype == 'base':
        bc = BCBase()
    elif bctype == 'beacon':
        bc = BCBeacon()
    else:
        print("No type", bctype)

    try:
        bc.start()
    except rospy.ROSInterruptException:
        pass
