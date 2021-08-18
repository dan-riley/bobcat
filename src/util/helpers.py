#!/usr/bin/env python
from __future__ import print_function
from cmath import rect, phase
import math

from geometry_msgs.msg import Point


def getDist(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)


def getDist2D(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)


def getYaw(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    return math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))


def getAngle(a, b, c):
    # Get the angle at point b between 3 points
    ab = Point()
    bc = Point()
    ab.x = b.x - a.x
    ab.y = b.y - a.y
    ab.z = b.z - a.z
    bc.x = c.x - b.x
    bc.y = c.y - b.y
    bc.z = c.z - b.z

    normalize(ab)
    normalize(bc)
    dot = ab.x * bc.x + ab.y * bc.y + ab.z * bc.z

    if dot > 1: dot = 1
    if dot < -1: dot = -1

    return math.acos(dot) * 180 / math.pi


def normalize(a):
    mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
    if not mag: return
    a.x = a.x / mag
    a.y = a.y / mag
    a.z = a.z / mag


def averagePose(history):
    pos = Point()
    yaw = 0
    for pose in history:
        pos.x = pos.x + pose.position.x
        pos.y = pos.y + pose.position.y
        pos.z = pos.z + pose.position.z
        yaw = yaw + rect(1, getYaw(pose.orientation))

    pos.x = pos.x / float(len(history))
    pos.y = pos.y / float(len(history))
    pos.z = pos.z / float(len(history))
    yaw = math.degrees(phase(yaw))

    return pos, yaw


def averagePosition(history):
    pos = Point()
    for position in history:
        pos.x = pos.x + position.x
        pos.y = pos.y + position.y
        pos.z = pos.z + position.z

    pos.x = pos.x / float(len(history))
    pos.y = pos.y / float(len(history))
    pos.z = pos.z / float(len(history))

    return pos


def angleDiff(a, b):
    # Computes a-b, preserving the correct sign (counter-clockwise positive angles)
    # All angles are in degrees
    a = (360000 + a) % 360
    b = (360000 + b) % 360
    d = a - b
    d = (d + 180) % 360 - 180
    return d


def getSeq(octomap):
    return octomap.header.seq
