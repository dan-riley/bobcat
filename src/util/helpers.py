#!/usr/bin/env python
from __future__ import print_function
from cmath import rect, phase
import math
import copy
import numpy as np

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


def positionToNP(position):
    return np.array([position.x, position.y, position.z])


def pointToPathDistance(pt, r0, r1):
    r01 = r1 - r0
    d = np.linalg.norm(r01)
    r01u = r01 / d
    r = pt - r0
    rid = np.dot(r, r01u)
    if rid > d:
        return r1
    elif rid < 0:
        return r0
    ri = r01u * ridP
    lpt = r0 + ri

    return np.linalg.norm(npos - nearPoint)


def lineDistance(a0, a1, b0, b1):
    A = a1 - a0
    B = b1 - b0
    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)

    _A = A / magA
    _B = B / magB

    cross = np.cross(_A, _B)
    denom = np.linalg.norm(cross)**2

    # Extended lines don't cross
    if not denom:
        d0 = np.dot(_A, (b0-a0))
        d1 = np.dot(_A, (b1-a0))
        if d0 <= 0 >= d1:
            if np.absolute(d0) < np.absolute(d1):
                return np.linalg.norm(a0-b0)
            return np.linalg.norm(a0-b1)

        elif d0 >= magA <= d1:
            if np.absolute(d0) < np.absolute(d1):
                return np.linalg.norm(a1-b0)
            return np.linalg.norm(a1-b1)

        return np.linalg.norm(((d0 * _A) + a0) - b0)

    # Extended lines cross
    t = b0 - a0
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA / denom
    t1 = detB / denom

    if t0 < 0:
        pA = a0
    elif t0 > magA:
        pA = a1
    else:
        pA = a0 + _A * t0

    if t1 < 0:
        pB = b0
    elif t1 > magB:
        pB = b1
    else:
        pB = b0 + _B * t1

    if t0 < 0 or t0 > magA:
        dot = np.dot(_B, pA - b0)
        if dot < 0:
            dot = 0
        elif dot > magB:
            dot = magB
        pB = b0 + _B * dot

    if t1 < 0 or t1 > magB:
        dot = np.dot(_A, pB - a0)
        if dot < 0:
            dot = 0
        elif dot > magA:
            dot = magA
        pA = a0 + _A * dot

    return np.linalg.norm(pA - pB)


def comparePaths(path1, path2, limit):
    # Check each segment in second path, so we can look the current robot position first
    i = 0
    descending = False
    while i < len(path2.poses) - 1:
        p20 = positionToNP(path2.poses[i].pose.position)
        p21 = positionToNP(path2.poses[i+1].pose.position)

        # Check each segment in first path
        j = 0
        prevDist = 0
        while j < len(path1.poses) - 1:
            p10 = positionToNP(path1.poses[j].pose.position)
            p11 = positionToNP(path1.poses[j+1].pose.position)
            ptDistance = lineDistance(p10, p11, p20, p21)
            if i == 0 and  ptDistance < prevDist:
                descending = True

            if descending and ptDistance < limit:
                # Path points toward path2 and is inside the limit
                return True
            elif i == 0 and j == 0 and ptDistance < limit / 2:
                # Robots are really close, so consider stopping
                return True

            prevDist = ptDistance
            j += 1
        i += 1

    return False


def truncatePath(input_path, pos):
    # Truncate a path starting from a position
    if not input_path.poses:
        return input_path
    # Don't want to change the path
    path = copy.deepcopy(input_path)
    i = 0
    dist1 = getDist(pos, path.poses[0].pose.position)
    mindist = dist1
    mini = 0
    while i < len(path.poses) - 1:
        dist0 = getDist(path.poses[i].pose.position, path.poses[i+1].pose.position)
        dist2 = getDist(pos, path.poses[i+1].pose.position)
        if (dist1 < dist2 and dist2 < dist0) or (dist2 < dist1 and dist1 < dist0):
            break
        else:
            if dist2 < mindist:
                mindist = dist2
                mini = i
            dist1 = dist2
            i += 1

    # If we got to the end, we need to go back to our min distance
    if i == len(path.poses) - 1:
        i = mini
    # Truncate the path
    path.poses = path.poses[i:]
    # Replace the first pose with the current position
    path.poses[0].pose.position = pos

    return path
