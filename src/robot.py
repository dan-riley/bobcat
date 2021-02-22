#!/usr/bin/env python
from __future__ import print_function
import math
import rospy

from util.helpers import getDist, getYaw, averagePosition, angleDiff
from monitors import BCMonitors
from behaviors import BCBehaviors
from actions import BCActions
from BOBCAT import BOBCAT


class BCRobot(BOBCAT, BCMonitors, BCBehaviors, BCActions):
    """ Initialize a multi-agent robot node """

    def __init__(self):
        # Get all of the parent class variables
        BOBCAT.__init__(self)

        # Distance to maintain goal point deconfliction
        self.deconflictRadius = rospy.get_param('bobcat/deconflictRadius', 2.5)
        # Time stopped to report if stuck
        self.stopCheck = rospy.get_param('bobcat/stopCheck', 30)

        self.startedMission = False
        self.initialPose = False
        self.history = []
        self.hislen = self.rate * 10  # How long the odometry history should be
        self.mode = 'Explore'
        self.stuck = 0
        self.commListen = True

        # Initialize all of the BOBCAT modules
        BCMonitors.__init__(self)
        BCActions.__init__(self)

    def updateHistory(self):
        # Check whether we've started the mission by moving 5 meters
        if not self.initialPose:
            self.initialPose = self.agent.odometry.pose.pose
        elif not self.startedMission:
            # Do this here so we only do this calculation until leaving the starting area
            if getDist(self.agent.odometry.pose.pose.position, self.initialPose.position) > 5:
                self.startedMission = True

        self.history.append(self.agent.odometry.pose.pose)
        if len(self.history) > self.hislen:
            self.history = self.history[-self.hislen:]

    ##### Main BOBCAT Execution #####
    def run(self):
        # Update our comm status for anyone who needs it
        self.comm_pub.publish(self.base.incomm)

        # Update movement history
        self.updateHistory()

        # Do status checks once we've started the mission
        if self.startedMission and self.agent.status != 'Stop' and 'A' not in self.id:
            if self.agent.goal.path.poses and len(self.history) == self.hislen:
                # Check if we've been stopped if we have a goal
                if (getDist(self.history[0].position, self.history[-1].position) < 0.5 and
                        abs(angleDiff(math.degrees(getYaw(self.history[0].orientation)),
                                      math.degrees(getYaw(self.history[-1].orientation)))) < 60):
                    self.stuck += 1
                    # Add the current goal to potential blacklist points
                    self.blgoals.append(self.agent.goal.pose.pose.position)
                else:
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

                            # Make sure it's not the origin
                            if not (avgGoal.x == 0 and avgGoal.y == 0 and avgGoal.z == 0):
                                self.addBlacklist(avgGoal)
                                self.blgoals = []

                    self.updateStatus('Stuck')
                    rospy.loginfo(self.id + ' has not moved!')

            elif not self.agent.goal.path.poses:
                # Report no path available
                self.updateStatus('No Path')
                # rospy.loginfo(self.id + ' no path!')

        checkBeacon = True
        # Manage the newest task sent
        if self.agent.guiAccept:
            if self.agent.guiTaskName == 'task':
                if self.agent.guiTaskValue == 'Explore' or self.agent.guiTaskValue == 'Start':
                    self.mode = 'Explore'
                elif self.agent.guiTaskValue == 'Home':
                    self.mode = 'Home'
                elif self.agent.guiTaskValue == 'Stop':
                    self.mode = 'Stop'
                elif self.agent.guiTaskValue == 'Goal':
                    self.mode = 'Goal'
                elif self.agent.guiTaskValue == 'Deploy':
                    self.deployBeacon(True, 'GUI Command')
                    checkBeacon = False
                    self.mode = 'Explore'

                # Disable the estop.  'Stop' will re-enable it
                self.stop_pub.publish(True)
            else:
                self.publishGUITask()

            self.agent.guiAccept = False

        # Check whether to drop a beacon, as long as we weren't commanded by the GUI
        if checkBeacon:
            self.beaconCheck()

        num_neighbors = 0
        # Time check for "current" neighbors.  Make sure we don't have negative time.
        if rospy.get_rostime() > rospy.Time(0) + self.commThreshold * 30:
            neighbor_check = rospy.get_rostime() - self.commThreshold * 30
        else:
            neighbor_check = rospy.get_rostime()

        # Get our neighbors' artifacts so we can deconflict reporting
        for neighbor in self.neighbors.values():
            self.artifactCheck(neighbor)

            # Count how many neighbors we have current goal information for deconfliction
            # Using 60 seconds for now
            # This is only accurate if times are relatively in sync (within 60-ish seconds)
            if neighbor.lastMessage > neighbor_check:
                num_neighbors += 1

        # Publish the number of neighbors that frontier exploration should consider
        self.num_pub.publish(num_neighbors)

        # Make sure our internal artifact list is up to date, and if we need to report
        self.artifactCheck(self.agent)
        self.artifactCheckReport()

        # Decide which goal to go to based on status in this precedence:
        # Report Artifacts
        # GUI Return Home
        # GUI Stop
        # GUI Goal Point
        # Explore
        if self.report:
            # Once we see the base has our latest artifact report we can stop going home
            if self.solo or self.base.lastArtifact == self.agent.lastArtifact:
                # Turn off reporting
                self.report = False
                for artifact in self.artifacts.values():
                    artifact.reported = True

                # Resume normal operation (check mode or explore)
                rospy.loginfo(self.id + ' resuming operation...')
            else:
                if self.agent.status != 'Report':
                    rospy.loginfo(self.id + ' return to report...')
                self.setGoalPoint('Report')
                # Don't check for other mode when in report
                return True

        # Check for a mode from GUI or MA, or explore normally
        if self.mode == 'Home':
            self.setGoalPoint('Home')
        elif self.mode == 'Stop':
            self.stop()
        elif self.mode == 'Deploy':
            rospy.loginfo(self.id + ' reverse deploy mode')
            if self.base.incomm:
                # Wait for a solid connection before dropping
                if self.regainBase > 5:
                    # Make sure there's no beacons already in the area
                    pose = self.agent.odometry.pose.pose
                    checkDist = self.junctionDist
                    dropBeacon = True
                    dropBeacon, nB, nDB = self.beaconDistCheck(pose, checkDist, dropBeacon)
                    if dropBeacon:
                        self.deployBeacon(True, 'Regain comms')
                    else:
                        # The bl_beacons should prevent this from occuring too many times
                        rospy.loginfo(self.id + ' beacon too close, cancelling drop')
                    self.mode = 'Explore'
                    self.regainBase = 0
                else:
                    self.regainBase += 1
            else:
                self.regainBase = 0
                self.updateStatus('Regain comms deploy')
                self.setGoalPoint('Home')
        elif self.mode == 'Goal':
            if (getDist(self.agent.odometry.pose.pose.position,
                        self.agent.guiGoalPoint.pose.position) < 1.0):
                rospy.loginfo(self.id + ' resuming exploration...')
                self.mode = 'Explore'
                # May want to add other options for tasks when it reaches the goal
                self.deconflictExplore()
            else:
                if self.agent.status != 'guiCommand':
                    rospy.loginfo(self.id + ' setting GUI Goal Point...')
                self.setGoalPoint('guiCommand')
        else:
            # Normal exploration with coordination
            self.deconflictExplore()

        # Have the aerial robots prevent maps from being merged until they're airborne
        # Useful particularly for marsupials
        # May consider using seqs to clear instead, as 'ignore' will cause the entire map
        # to get transmitted once the robot launches, but seqs means it was transmitted and
        # then thrown away...not sure best approach
        if self.isAerial and not self.launch_status and not self.agent.reset.ignore:
            rospy.loginfo('ignore aerial maps until launched')
            self.agent.reset.stamp = rospy.get_rostime()
            self.agent.reset.agent = self.id
            self.agent.reset.ignore = True
            self.agent.reset.robots = True
            self.agent.guiStamp = self.agent.reset.stamp
        elif self.isAerial and self.launch_status and self.agent.reset.ignore:
            rospy.loginfo('aerial robot launched, stop ignoring maps')
            self.agent.reset.agent = self.id
            self.agent.reset.ignore = False
            self.agent.reset.robots = True
            self.agent.reset.stamp = rospy.get_rostime()
            self.agent.guiStamp = self.agent.reset.stamp

        return True
