#!/usr/bin/env python
from __future__ import print_function
from cmath import rect, phase
import math

from geometry_msgs.msg import Point
from bobcat.msg import Goal


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


def subsample(goal):
    pubgoal = Goal()
    pubgoal.pose = goal.pose
    pubgoal.path.header.frame_id = goal.path.header.frame_id

    for i, pose in enumerate(goal.path.poses):
        if i % 20 == 0:
            pubgoal.path.poses.append(pose)

    if pubgoal.path.poses and pubgoal.path.poses[-1] != goal.path.poses[-1]:
        pubgoal.path.poses.append(goal.path.poses[-1])

    return pubgoal

def getSeq(octomap):
    return octomap.header.seq
