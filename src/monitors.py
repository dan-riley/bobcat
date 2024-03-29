#!/usr/bin/env python
from __future__ import print_function
import math
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from marble_origin_detection_msgs.msg import OriginDetectionStatus
from estop_msgs.msg import BaseStatus
from sensor_msgs.msg import BatteryState

from util.helpers import getDist, getYaw, averagePose, averagePosition, angleDiff, comparePointToPath, comparePaths, truncatePath


class BCMonitors():
    """ Establish available BOBCAT Monitors """

    def __init__(self):

        # Distance to maintain goal point deconfliction
        self.deconflictRadius = rospy.get_param('bobcat/deconflictRadius', 2.5)
        # Time stopped to report if stuck
        self.stopCheck = rospy.get_param('bobcat/stopCheck', 30)
        # Mapping range, to decide what "long distance goals" are
        self.mappingRange = rospy.get_param('bobcat/mappingRange', 5.0)
        # Distance from Anchor to drop beacons automatically
        self.maxAnchorDist = rospy.get_param('bobcat/anchorDropDist', 100)
        # Distance to drop beacons automatically
        self.maxDist = rospy.get_param('bobcat/dropDist', 30)
        # Minimum distance between junctions before dropping another beacon
        self.junctionDist = rospy.get_param('bobcat/junctionDist', 10)
        # Minimum distance between beacons when dropping due to lost comms
        self.redeployDist = rospy.get_param('bobcat/redeployDist', 10)
        # Whether to use turn detection to drop beacons
        self.turnDetect = rospy.get_param('bobcat/turnDetect', True)
        # Whether this agent should delay their drop so the trailing robot can
        self.delayDrop = rospy.get_param('bobcat/delayDrop', False)
        # Whether to backtrack to deploy a beacon
        self.reverseDropEnable = rospy.get_param('bobcat/reverseDrop', False)
        # How many artifacts are needed before reporting
        self.maxNewArtifacts = rospy.get_param('bobcat/maxNewArtifacts', 5)
        # Maximum amount of time to wait until reporting
        self.maxReportTime = rospy.get_param('bobcat/maxReportTime', 300)

        # Static anchor position
        self.anchorPos = Point()
        self.anchorPos.x = rospy.get_param('bobcat/anchorX', 1.0)
        self.anchorPos.y = rospy.get_param('bobcat/anchorY', 0.0)
        self.anchorPos.z = rospy.get_param('bobcat/anchorZ', 0.1)

        self.bl_beacons = []
        self.minAnchorDist = 10  # Minimum distance before a beacon is ever dropped
        self.beaconCommLost = 0
        self.beaconCommLostPos = Point()
        self.dropReason = ''
        self.checkReverse = True
        self.neighborWait = 0
        self.stuck = 0
        self.stuckPose = Pose()
        self.paused = False
        self.ignoreStopCommand = False
        self.lastStopCommand = rospy.get_rostime() + rospy.Duration(1)
        self.checkCarefulTime = rospy.get_rostime() + rospy.Duration(5)
        self.blacklistResetTime = rospy.Time()
        self.newArtifactTime = rospy.Time()
        self.numNewArtifacts = 0
        self.guiStopTime = rospy.get_rostime() + rospy.Duration(3600) # One hour, until set by GUI

        # Monitor outputs
        self.replan = False
        self.report = False
        self.deployBeacon = False
        self.reverseDrop = False
        self.planner_status = True
        self.exploreToGoal = False
        self.launch_status = True
        self.isAerial = False
        self.blacklistUpdated = False
        self.guiBehavior = 'stop' # Start the robot in stop mode
        self.lastGuiBehavior = None
        self.nearbyRobot = False
        self.beaconDeployed = False

        # Subscribers for some Monitors
        waitTopic = rospy.get_param('bobcat/waitTopic', 'origin_detection_status')
        stopTopic = rospy.get_param('bobcat/stopTopic', 'estop')
        self.wait_sub = rospy.Subscriber(waitTopic, OriginDetectionStatus, self.WaitMonitor)
        self.stop_sub = rospy.Subscriber(stopTopic, Bool, self.StopMonitor)
        self.estop_sub = rospy.Subscriber('estop_cmd', Bool, self.EStopMonitor)
        self.deploy_sub = rospy.Subscriber('deploy', Bool, self.DeployMonitor)
        self.input_sub = rospy.Subscriber('forceTask', String, self.InputMonitor)
        self.planner_sub = rospy.Subscriber('planner_status', Bool, self.PlannerMonitor)
        self.launch_sub = rospy.Subscriber('velocity_controller/enable', Bool, self.LaunchMonitor)
        if self.useSimComms or self.useVirtual:
            self.battery_sub = rospy.Subscriber('battery_state', BatteryState, self.BatterySimMonitor)
        else:
            self.battery_sub = rospy.Subscriber('base_state', BaseStatus, self.BatteryMonitor)
            self.battery_spot_sub = rospy.Subscriber('fp_state', BaseStatus, self.BatteryMonitor)

    def WaitMonitor(self, data):
        if data.status > 0:
            self.wait = False

    def StopMonitor(self, data):
        # Ignore any commands if inhibited, or if it's repeated too fast
        if self.ignoreStopCommand or rospy.get_rostime() < self.lastStopCommand:
            self.ignoreStopCommand = False
            return

        # Timeout for fast repeated commands
        self.lastStopCommand = rospy.get_rostime() + rospy.Duration(1)

        # This technically monitors the estop whether sent from Bobcat or externally,
        # but the main point is to capture joystick commands to stop the robot
        if data.data and self.stopCommand:
            if self.guiBehavior != 'stop':
                self.lastGuiBehavior = self.guiBehavior
                self.guiBehavior = 'stop'
        elif self.guiBehavior == 'stop':
            # If we were stopped, we should go back to whatever the last thing requested was
            self.guiBehavior = self.lastGuiBehavior

    def EStopMonitor(self, data):
        if not self.ignoreStopCommand and data.data:
            # Republish only hardware estop 'stop' commands to the software estop so they match
            self.stop_pub.publish(data.data and self.stopCommand)

    def DeployMonitor(self, data):
        if data.data:
            self.beaconDeployed = True

    def InputMonitor(self, data):
        self.agent.guiTaskName = 'task'
        self.agent.guiTaskValue = data.data
        self.agent.guiAccept = True

    def PlannerMonitor(self, data):
        self.planner_status = data.data
        if not self.planner_status:
            self.updateStatus('Unable to plan')

    def LaunchMonitor(self, data):
        self.isAerial = True
        self.launch_status = data.data

    def PoseGraphMonitor(self):
        if rospy.get_rostime() > self.lastPoseGraphTime + rospy.Duration(10):
            # Save the time
            self.lastPoseGraphTime = rospy.get_rostime()
            # Get our subsampled and compressed path to save for transmission
            self.agent.poseGraphCompressed = self.compressPath(self.agent.poseGraph, self.ssDistancePoseGraph, self.ssAnglePoseGraph, True)

    def BatteryMonitor(self, data):
        vbat = [x for x in data.vbat if x > 10]
        if vbat:
            self.agent.battery = min(vbat)
        else:
            self.agent.battery = 0

    def BatterySimMonitor(self, data):
        self.agent.battery = data.voltage

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

    def ReverseDropMonitor(self):
        if not self.numBeacons:
            self.reverseDrop = False
            return

        # Reset the ability to check for reverse drop if we ever regain comms
        if self.base.incomm:
            self.checkReverse = True
            self.beaconCommLost = 0

        threshold = self.commThreshold * 2 + rospy.Duration(1)
        pose = self.agent.odometry.pose.pose
        # Check for reverse if we haven't disabled due to a blacklist, if not in comms,
        # and if we're not currently trying to reverse drop
        if self.checkReverse and not self.base.incomm and not self.reverseDrop:
            if not self.beaconCommLost:
                self.beaconCommLostPos = pose.position
            self.beaconCommLost += 1
            # If we're not talking to the base station, attempt to reverse drop
            # Wait longer the more beacons we drop to account for latency
            if (self.beaconCommLost > threshold.secs + 2 * len(self.beaconsArray) and
                    getDist(pose.position, self.beaconCommLostPos) > 3):
                self.reverseDrop = True
                # Check if we've already attempted a reverse drop in this area
                # Sometimes there are deadzones and this can cause a loop if not accounted for
                for bl in self.bl_beacons:
                    if getDist(pose.position, bl) < self.redeployDist:
                        self.reverseDrop = False
                        # Disable checking again unless we regain comms at some point
                        self.checkReverse = False
                        rospy.loginfo(self.id + ' skipping reverse drop due to previous try')
                        break

                # Add to the list of previously tried positions and save our return goal
                if self.reverseDrop:
                    self.bl_beacons.append(pose.position)
                    self.agent.guiGoalPoint = self.agent.exploreGoal

                # Reset the counter so we don't attempt again right away
                self.beaconCommLost = 0

        # Figure out how many messages we need to consider us in good enough comms to drop
        # Assume every beacon added reduces our effectiveness, even from other agents
        regainNeeded = self.commThreshold.secs + 1 - len(self.beaconsArray)
        if regainNeeded < 1:
            regainNeeded = 1
        # If we've regained comms and were trying to reverse drop, check if we can
        if (self.reverseDrop and self.base.incomm and rospy.get_rostime() +
                rospy.Duration(regainNeeded - self.base.regain + 1) <
                self.base.regainTime + threshold):
            checkDist = self.redeployDist

            # If we've gotten a couple of new messages, stop and check comms
            if self.base.regain > 1 and not self.paused:
                rospy.loginfo(self.id + ' checking beacon comms')
                self.base.resetRegain()
                self.pause()
                return

            if self.base.regain > regainNeeded:
                # Make sure there's no beacons already in the area
                dropBeacon = True
                dropBeacon, nB, nDB = self.beaconDistCheck(pose, checkDist, dropBeacon)
                if dropBeacon:
                    self.dropReason = 'Regain comms'
                    self.deployBeacon = True
                else:
                    # The bl_beacons should prevent this from occuring too many times
                    rospy.loginfo(self.id + ' beacon too close, cancelling drop')
                self.reverseDrop = False
                self.exploreToGoal = True
                self.move(True)
            else:
                if getDist(pose.position, self.anchorPos) < 10:
                    # If we're close to anchor, comms must just be too bad to keep trying to drop
                    # Drop one anyway if there's not already one nearby just in case it helps
                    dropBeacon = True
                    dropBeacon, nB, nDB = self.beaconDistCheck(pose, checkDist, dropBeacon)
                    if dropBeacon:
                        self.dropReason = 'Near anchor'
                        self.deployBeacon = True
                    else:
                        rospy.loginfo(self.id + ' anchor too close, cancelling drop')
                    self.reverseDrop = False
                    self.exploreToGoal = True
                    self.move(True)
        elif self.reverseDrop:
            self.base.resetRegain()
            self.move(True)

    def BeaconMonitor(self):
        # Don't check if we're already deploying for some reason
        if self.deployBeacon or self.reverseDrop or self.lastBeacon:
            return

        # Check if we need to drop a beacon if we have any beacons to drop
        if self.numBeacons > 0:
            pose = self.agent.odometry.pose.pose
            dropBeacon = False
            dropReason = ''

            # We're connected to the mesh, either through anchor or beacon(s)
            if self.base.incomm and self.base.regain > 10:
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
                        # Set the monitor so the deployment should get executed
                        self.dropReason = dropReason
                        self.deployBeacon = True

    def ArtifactMonitor(self):
        # Identify our report so we can track that the base station has seen it
        if self.numNewArtifacts:
            # Make sure we have enough new artifacts OR enough time has passed
            # Always report once the robot has reached the time set by the supervisor
            if ((self.numNewArtifacts >= self.maxNewArtifacts) or
                (rospy.get_rostime() >
                 self.newArtifactTime + rospy.Duration(self.maxReportTime)) or
                (rospy.get_rostime() > self.guiStopTime)):
                self.report = True

            # Once we see the base has our latest artifact report we can stop going home
            if self.solo or self.base.lastArtifact == self.agent.lastArtifact:
                # Turn off reporting
                self.report = False
                self.numNewArtifacts = 0
                for artifact in self.artifacts.values():
                    artifact.reported = True

                # Resume normal operation
                rospy.loginfo(self.id + ' artifacts reported')
            elif self.base.incomm and self.base.lastArtifact != self.agent.lastArtifact:
                self.agent.updateHash()

    def StuckMonitor(self):
        self.blacklistUpdated = False
        if self.agent.goal.path.poses and len(self.history) == self.hislen:
            # Clear the blacklist every two minutes
            if self.blacklist.points and rospy.get_rostime() > self.blacklistResetTime:
                self.blacklist.points = []
                rospy.loginfo('reset blacklist')

            # Check if we've been stopped if we have a goal
            if (getDist(self.stuckPose.position, self.history[-1].position) < 0.5 and
                    abs(angleDiff(math.degrees(getYaw(self.stuckPose.orientation)),
                                  math.degrees(getYaw(self.history[-1].orientation)))) < 60):
                self.stuck += 1
                # Add the current goal to potential blacklist points
                self.blgoals.append(self.agent.goal.pose.pose.position)
            else:
                self.stuckPose = self.history[0]
                self.stuck = 0
                self.blgoals = []

            # If stuck, report and append to blacklist so we don't try to go here again
            if self.stuck >= self.stopCheck:
                # Only add to the blacklist at the stopCheck intervals,
                # or else they get added too often

                if self.stuck % self.stopCheck == 0:
                    # Get the average goal position
                    avgGoal = averagePosition(self.blgoals)
                    # Remove outliers
                    newgoals = []
                    for goal in self.blgoals:
                        if getDist(goal, avgGoal) < self.deconflictRadius * 2:
                            newgoals.append(goal)

                    # Get the new average goal position, and only if there are enough
                    if len(newgoals) > self.hislen / 2:
                        avgGoal = averagePosition(newgoals)

                        # Make sure we're exploring and we're not already close to the goal
                        if (self.lastBehavior.name == 'Explore' and
                                getDist(self.agent.odometry.pose.pose.position,
                                        self.agent.goal.pose.pose.position) > 0.5):
                            self.addBlacklist(avgGoal)
                            self.blgoals = []

                    # Tell the planner to replan and blacklist
                    self.blacklistUpdated = 'newBlacklist'

                self.updateStatus('Stuck')
                rospy.loginfo(self.id + ' has not moved!')

        elif not self.agent.goal.path.poses:
            # Report no path available
            self.updateStatus('No Path')

    def NeighborMonitor(self):
        # Check if there's another robot close by and we should just stop until they're clear
        self.nearbyRobot = False
        curpos = self.agent.odometry.pose.pose.position
        path = truncatePath(self.agent.goal.path, curpos)

        for neighbor in self.neighbors.values():
            # Ignore the neighbor if we don't have current data, which should've come direct
            if rospy.get_rostime() > neighbor.lastDirectMessage + rospy.Duration(5):
                continue

            # First check to see if the neighbor is in the vicinity before bothering with path
            npos = neighbor.odometry.pose.pose.position
            curToNPos = getDist(curpos, npos)
            if curToNPos < self.deconflictRadius * 4:
                # Only use the path starting at the robot position
                npath = truncatePath(neighbor.goal.path, npos)
                # Check if our path points at their path and gets close to theirs
                if comparePaths(path, npath, self.deconflictRadius):
                    # Check if they're pointed at us and decide who stops
                    if comparePaths(npath, path, self.deconflictRadius):
                        # Break tie by ID.  H02 will stop for H01, and H01 will stop for D02.
                        if self.id > neighbor.id:
                            self.nearbyRobot = True
                            break
                    else:
                        # We could run into them if we catch up, so wait
                        self.nearbyRobot = True
                        break

        # Reset our waiting timer when there's no conflict
        if not self.nearbyRobot:
            self.neighborWait = 0

    def CarefulMonitor(self):
        # If a beacons or robot are within localization error distance tell planner to be careful
        careful = False
        curpos = self.agent.odometry.pose.pose.position
        path = truncatePath(self.agent.goal.path, curpos)
        for beacon in self.beacons.values():
            if beacon.active and getDist(beacon.pos, curpos) < 3:
                if comparePointToPath(beacon.pos, path, 1.5):
                    careful = True
                    break

        if careful and rospy.get_rostime() > self.checkCarefulTime:
            # Tell the planner to look for dynamic obstacles in this area
            rospy.loginfo(self.id + ' beacon nearby, plan carefully')
            self.task_pub.publish('careful')
            self.checkCarefulTime = rospy.get_rostime() + rospy.Duration(5)

    def GUIMonitor(self):
        # Manage the newest task sent
        if self.agent.guiAccept:
            self.lastGuiBehavior = self.guiBehavior
            if self.agent.guiTaskName == 'task':
                if self.agent.guiTaskValue == 'Explore' or self.agent.guiTaskValue == 'Start':
                    # Request a new goal from the planner if we were already exploring
                    if self.guiBehavior == None:
                        self.replan = 'gui'
                    # Since Explore is the default behavior, these commands just reset the
                    # robot to normal autonomous mode
                    self.guiBehavior = None
                elif self.agent.guiTaskValue == 'Home':
                    self.guiBehavior = 'home'
                elif self.agent.guiTaskValue == 'Stop':
                    self.guiBehavior = 'stop'
                elif self.agent.guiTaskValue == 'Goal':
                    self.guiBehavior = 'goToGoal'
                elif self.agent.guiTaskValue == 'Deploy':
                    self.guiBehavior = 'deployBeacon'
                    self.dropReason = 'GUI Command'
                    self.deployBeacon = True

                self.exploreToGoal = False
                rospy.loginfo(self.id + ' received new GUI Task ' + self.agent.guiTaskValue)
            elif self.agent.guiTaskName == 'setGUITime':
                self.guiStopTime = rospy.Time(int(self.agent.guiTaskValue) / 1000000000)
                rospy.loginfo(self.id + ' set mission stop time to ' + str(self.guiStopTime))
                self.updateStatus('Set Stop Time')
            elif self.agent.guiTaskName == 'setBeacons':
                self.myBeacons = self.agent.guiTaskValue.split(',')
                self.numBeacons = len(self.myBeacons) if self.myBeacons[0] != '' else 0
                self.startBeacons = self.numBeacons
                for beacon in self.myBeacons:
                    self.beacons[beacon].owner = True
                rospy.loginfo(self.id + ' loaded beacons ' + str(self.myBeacons))
                self.updateStatus('Loaded Beacons')
            else:
                # Publish a boolean to the given topic for direct control
                # Shouldn't really be in a Monitor but for now the best place for it
                self.publishGUITask()

            self.agent.guiAccept = False
