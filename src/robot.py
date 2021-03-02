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
        self.commListen = True

        # Initialize all of the BOBCAT modules
        BCMonitors.__init__(self)
        BCActions.__init__(self)

        # Setup Objectives
        self.objectives = {}
        self.objectives['explore'] = objectives.Explore(self)
        self.objectives['report'] = objectives.ReportArtifacts(self)
        self.objectives['input'] = objectives.Input(self)
        self.objectives['maintainComms'] = objectives.MaintainComms(self)
        self.objectives['extendComms'] = objectives.ExtendComms(self)

        # Add the priority to specified Objectives
        # Need to make this a launch file parameter to loop
        self.objectives['report'].setPriority()
        self.objectives['input'].setPriority()
        self.objectives['extendComms'].setPriority()

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
            if getDist(self.agent.odometry.pose.pose.position, self.initialPose.position) > 5:
                self.startedMission = True

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
        ### End Message Aggregation ###

        ### Start Monitor updates ###
        if self.startedMission and self.agent.status != 'Stop' and 'A' not in self.id:
            self.StuckMonitor()
        self.BeaconMonitor()
        if self.reverseDropEnable:
            self.ReverseDropMonitor()
        self.ArtifactMonitor()
        self.GUIMonitor()
        ### End Monitor updates ###

        # Update Objective Weights
        for objective in self.objectives.values():
            objective.evaluate()

        # Update Behavior Scores and find the highest one
        maxScore = 0
        execBehavior = None
        for behavior in self.behaviors.values():
            behavior.evaluate()
            if behavior.score > maxScore:
                maxScore = behavior.score
                execBehavior = behavior
            elif behavior.score == maxScore:
                if execBehavior:
                    print("we have a tie!", behavior.name, execBehavior.name)

        # Execute the chosen Behavior
        print('executing', execBehavior.name)
        execBehavior.execute()

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
