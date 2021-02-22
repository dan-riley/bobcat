#!/usr/bin/env python
from __future__ import print_function
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from marble_origin_detection_msgs.msg import OriginDetectionStatus

from util.helpers import getDist, averagePose, angleDiff


class BCMonitors():
    """ Establish available BOBCAT Monitors """

    def __init__(self):
        # Distance from Anchor to drop beacons automatically
        self.maxAnchorDist = rospy.get_param('bobcat/anchorDropDist', 100)
        # Distance to drop beacons automatically
        self.maxDist = rospy.get_param('bobcat/dropDist', 30)
        # Minimum distance between junctions before dropping another beacon
        self.junctionDist = rospy.get_param('bobcat/junctionDist', 10)
        # Whether to use turn detection to drop beacons
        self.turnDetect = rospy.get_param('bobcat/turnDetect', True)
        # Whether this agent should delay their drop so the trailing robot can
        self.delayDrop = rospy.get_param('bobcat/delayDrop', False)
        # Whether to backtrack to deploy a beacon
        self.reverseDropEnable = rospy.get_param('bobcat/reverseDrop', False)
        self.reverseDrop = self.reverseDropEnable

        # Static anchor position
        self.anchorPos = Point()
        self.anchorPos.x = rospy.get_param('bobcat/anchorX', 1.0)
        self.anchorPos.y = rospy.get_param('bobcat/anchorY', 0.0)
        self.anchorPos.z = rospy.get_param('bobcat/anchorZ', 0.1)

        self.bl_beacons = []
        self.minAnchorDist = 10  # Minimum distance before a beacon is ever dropped
        self.report = False
        self.newStatus = False
        self.statusCount = 0
        self.beaconCommLost = 0
        self.planner_status = True
        self.launch_status = True
        self.isAerial = False

        waitTopic = rospy.get_param('bobcat/waitTopic', 'origin_detection_status')
        self.wait_sub = rospy.Subscriber(waitTopic, OriginDetectionStatus, self.WaitMonitor)
        self.planner_sub = rospy.Subscriber('planner_status', Bool, self.PlannerMonitor)
        self.launch_sub = rospy.Subscriber('velocity_controller/enable', Bool, self.LaunchMonitor)

    def WaitMonitor(self, data):
        if data.status > 0:
            self.wait = False

    def PlannerMonitor(self, data):
        self.planner_status = data.data
        if not self.planner_status:
            self.updateStatus('Unable to plan')

    def LaunchMonitor(self, data):
        self.isAerial = True
        self.launch_status = data.data

    def getStatus(self):
        # Clear out the additional status messages every so often
        if self.statusCount > 5:
            self.newStatus = False
            self.statusCount = 0
        else:
            self.statusCount += 1

        if self.newStatus:
            status = self.agent.status + '+++' + self.newStatus
        else:
            status = self.agent.status

        return status

    def beaconDistCheck(self, pose, checkDist, dropBeacon):
        numBeacons = 0
        numDistBeacons = 0
        for beacon in self.beacons.values():
            if beacon.active:
                # Count the beacons we know about, and check distance
                numBeacons = numBeacons + 1
                dist = getDist(pose.position, beacon.pos)

                # Count how many beacons are past max range
                if dist > checkDist:
                    numDistBeacons = numDistBeacons + 1
                    dropBeacon = True

        # Cancel the drop if we have more than one beacon in range
        if numBeacons - numDistBeacons > 0:
            dropBeacon = False

        return dropBeacon, numBeacons, numDistBeacons

    def beaconCheck(self):
        # Check if we need to drop a beacon if we have any beacons to drop
        if self.numBeacons > 0:
            pose = self.agent.odometry.pose.pose
            dropBeacon = False
            dropReason = ''

            # We're connected to the mesh, either through anchor or beacon(s)
            if self.base.incomm:
                self.beaconCommLost = 0
                # Reset to global config so we can manipulate after cancelled drops
                # This should re-enable dropping if we re-gain comms after a cancelled drop
                self.reverseDrop = self.reverseDropEnable

                # Once we pass the maxDist we could set a flag so we don't keep recalculating this
                anchorDist = getDist(pose.position, self.anchorPos)
                # Beacon distance based drop only kicks in once out of anchor range
                checkDist = self.maxAnchorDist

                # If we're too close (like for the initial node drop), never drop a beacon
                if anchorDist < self.minAnchorDist:
                    return

                # If we're at end of anchor range, drop beacon
                if anchorDist > self.maxAnchorDist and not self.delayDrop:
                    dropBeacon = True
                    dropReason = 'anchor distance'
                    checkDist = self.maxDist

                # Always drop a beacon if we're at a node and we're in comm
                # If beacons are strong enough may want to restrict distance
                if self.agent.atnode.data:
                    dropBeacon = True
                    dropReason = 'at junction'
                    checkDist = self.junctionDist
                # Check if it looks like we're going around a corner
                elif self.turnDetect and len(self.history) == self.hislen:
                    pos1, yaw1 = averagePose(self.history[:int(0.4 * self.hislen)])
                    pos2, yaw2 = averagePose(self.history[int(0.6 * self.hislen):])

                    # Check that we've turned and moved far enough, over history and last second
                    # Will need to retune these for real vehicle dynamics
                    if (getDist(pos1, pos2) > 4 and abs(angleDiff(yaw1, yaw2)) > 30 and
                            getDist(self.history[-2].position, self.history[-1].position) > 0.5):
                        dropBeacon = True
                        dropReason = 'at turn'
                        checkDist = self.junctionDist + 5

                dropBeacon, numBeacons, numDistBeacons = self.beaconDistCheck(pose, checkDist, dropBeacon)

                if numDistBeacons > 0 and (dropReason == '' or dropReason == 'anchor distance'):
                    dropReason = 'beacon distance'

                # Prevent dropping after returning home after the first beacon drop
                # TODO look at the angle between anchor and first beacon and calculate positions
                # This only works for straight departure now
                if numBeacons > 0 and anchorDist < self.maxAnchorDist and pose.position.y < 1 and pose.position.y > 1:
                    dropBeacon = False

                if dropBeacon:
                    if self.delayDrop:
                        self.delayDrop = False
                    else:
                        self.deployBeacon(True, dropReason)
            elif self.reverseDrop:
                self.beaconCommLost += 1
                # If we're not talking to the base station, attempt to reverse drop
                if self.beaconCommLost > 5:
                    # Check if we've already attempted a reverse drop in this area
                    # Sometimes there are deadzones and this can cause a loop if not accounted for
                    for bl in self.bl_beacons:
                        if getDist(pose.position, bl) < self.junctionDist:
                            self.reverseDrop = False
                            rospy.loginfo(self.id + ' skipping reverse drop due to previous try')
                            break

                    # Change the mode to deploy, and add to the list of previously tried positions
                    if self.reverseDrop:
                        self.mode = 'Deploy'
                        self.bl_beacons.append(pose.position)

                    # Reset the counter so we don't attempt again right away
                    self.beaconCommLost = 0

    def artifactCheckReport(self):
        # If we didn't add anything new, check if any still need reported
        if not self.report:
            for artifact in self.artifacts.values():
                if artifact.agent_id == self.id and not artifact.reported:
                    self.report = True
                    break

        # Identify our report so we can track that the base station has seen it
        if self.report:
            rospy.loginfo('will report...')
