#!/usr/bin/env python
from __future__ import print_function
import rospy

from util.helpers import getDist
from monitors import BCMonitors
from actions import BCActions
import objectives, behaviors
from BOBCAT import BOBCAT


class BCRobot(BOBCAT, BCMonitors, BCActions):
    """ Initialize a multi-agent robot node """

    def __init__(self):
        # Get all of the parent class variables
        BOBCAT.__init__(self)

        self.startedMission = False
        self.initialPose = False
        self.history = []
        self.hislen = self.rate * 10  # How long the odometry history should be
        self.newStatus = False
        self.statusCount = 0
        self.guiBehavior = None
        self.lastGuiBehavior = None
        self.commListen = True
        self.debugWeights = False
        self.lastBehavior = None

        # Initialize all of the BOBCAT modules
        BCMonitors.__init__(self)
        BCActions.__init__(self)

        # Setup Objectives
        # Number of priority levels (same as lowest priority number)
        # Need to make this, and the arguments to the objectives, launch file parameters
        self.numPriorities = 3
        self.objectives = {}
        self.objectives['explore'] = objectives.Explore(self, 2)
        self.objectives['report'] = objectives.ReportArtifacts(self, 1)
        self.objectives['input'] = objectives.Input(self, 1)
        self.objectives['maintainComms'] = objectives.MaintainComms(self, 3)
        self.objectives['extendComms'] = objectives.ExtendComms(self, 1)

        # Setup Behaviors
        self.behaviors = {}
        self.behaviors['explore'] = behaviors.Explore(self)
        self.behaviors['goToGoal'] = behaviors.GoToGoal(self)
        self.behaviors['home'] = behaviors.GoHome(self)
        self.behaviors['stop'] = behaviors.Stop(self)
        self.behaviors['deployBeacon'] = behaviors.DeployBeacon(self)

    def updateHistory(self):
        # Check whether we've started the mission by moving 5 meters
        if not self.initialPose:
            self.initialPose = self.agent.odometry.pose.pose
        elif not self.startedMission:
            # Do this here so we only do this calculation until leaving the starting area
            if getDist(self.agent.odometry.pose.pose.position, self.initialPose.position) > 1:
                self.startedMission = True
                self.ignoreStopCommand = False

        self.history.append(self.agent.odometry.pose.pose)
        if len(self.history) > self.hislen:
            self.history = self.history[-self.hislen:]

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

    def delayAerialMaps(self):
        # Have the aerial robots prevent maps from being merged until they're airborne
        # Useful particularly for marsupials
        # May consider using seqs to clear instead, as 'ignore' will cause the entire map
        # to get transmitted once the robot launches, but seqs means it was transmitted and
        # then thrown away...not sure best approach
        if not self.launch_status and not self.agent.reset.ignore:
            rospy.loginfo('ignore aerial maps until launched')
            self.agent.reset.stamp = rospy.get_rostime()
            self.agent.reset.agent = self.id
            self.agent.reset.ignore = True
            self.agent.reset.robots = True
            self.agent.guiStamp = self.agent.reset.stamp
        elif self.launch_status and self.agent.reset.ignore:
            rospy.loginfo('aerial robot launched, stop ignoring maps')
            self.agent.reset.agent = self.id
            self.agent.reset.ignore = False
            self.agent.reset.robots = True
            self.agent.reset.stamp = rospy.get_rostime()
            self.agent.guiStamp = self.agent.reset.stamp

    ##### Main BOBCAT Execution #####
    def run(self):
        # Update our comm status for anyone who needs it
        self.comm_pub.publish(self.base.incomm)

        # Update movement history
        self.updateHistory()

        ### Start Message Aggregation ###
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

        # Wait until controller is enabled on air vehicle so maps don't transmit
        if self.isAerial:
            self.delayAerialMaps()

        ### End Message Aggregation ###

        ### Start Monitor updates ###
        if self.startedMission and self.agent.status != 'Stop':
            self.StuckMonitor()
        self.BeaconMonitor()
        if self.reverseDropEnable:
            self.ReverseDropMonitor()
        self.ArtifactMonitor()
        self.GUIMonitor()
        ### End Monitor updates ###

        # Update Objective Weights
        if self.debugWeights:
            rospy.loginfo('')
            rospy.loginfo('Objectives:')
        for objective in self.objectives.values():
            objective.evaluate()
            if self.debugWeights:
                rospy.loginfo(str(objective.__class__) + ' ' + str(objective.weight))

        # Update Behavior Scores and find the highest one
        maxScore = 0
        execBehavior = None
        if self.debugWeights:
            rospy.loginfo('')
            rospy.loginfo('Behaviors:')
        for behavior in self.behaviors.values():
            behavior.evaluate()
            if behavior.score > maxScore:
                maxScore = behavior.score
                execBehavior = behavior
            elif behavior.score == maxScore:
                if execBehavior and self.lastBehavior:
                    if behavior == self.lastBehavior:
                        execBehavior = behavior
                        print("tie, executing last behavior!")
                    else:
                        print("we have a tie!", behavior.name, execBehavior.name, self.lastBehavior.name)

            if self.debugWeights:
                rospy.loginfo(behavior.name + ' ' + str(behavior.score))

        # Execute the chosen Behavior
        if self.debugWeights:
            rospy.loginfo('')
            rospy.loginfo('executing ' + execBehavior.name)
        self.lastBehavior = execBehavior
        execBehavior.execute()

        return True
