#!/usr/bin/env python
from __future__ import print_function
import rospy

from util.helpers import getDist


class DefaultBehavior():
    """ Default Behavior object to extend for others """

    def __init__(self, agent):
        self.a = agent
        self.score = 0
        self.name = self.__class__.__name__
        self.monitors = []
        self.objectives = []

    def evaluate(self):
        return True

    def execute(self):
        self.score += 1

        return True


class DeployBeacon(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['ReverseDrop', 'HumanInput']
        self.objectives = ['ExtendComms', 'Input']

    def evaluate(self):
        self.score = 0
        # Inhibit the behavior if the Monitors say we should be reverse deploying
        if not self.a.reverseDrop:
            self.score = self.a.objectives['extendComms'].weight
        if self.a.guiBehavior == 'deployBeacon':
            self.score += self.a.objectives['input'].weight

        # Force continuation of behavior if a deployment has already started
        if self.a.lastBeacon:
            self.score = 2000

    def execute(self):
        self.a.dropBeacon()


class Explore(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['ExploreToGoal', 'HumanInput']  # Implict based on implementation
        self.objectives = ['FindArtifacts', 'Input']  # Implicit based on implementation

    def evaluate(self):
        self.score = 0
        if not self.a.exploreToGoal:
            self.score = self.a.objectives['findArtifacts'].weight

    def execute(self):
        self.a.explore()


class GoToGoal(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['ExploreToGoal', 'HumanInput']
        self.objectives = ['FindArtifacts', 'Input']

    def evaluate(self):
        self.score = 0
        if self.a.exploreToGoal:
            self.score = self.a.objectives['findArtifacts'].weight
        if self.a.guiBehavior == 'goToGoal':
            self.score = self.a.objectives['input'].weight

    def execute(self):
        # This part should probably be in a Monitor instead
        if (getDist(self.a.agent.odometry.pose.pose.position,
                    self.a.agent.guiGoalPoint.pose.position) < 2.0):
            rospy.loginfo(self.a.id + ' resuming exploration...')
            self.a.guiBehavior = None
            self.a.exploreToGoal = False
            # May want to add other options for tasks when it reaches the goal
            self.a.behaviors['explore'].execute()
        else:
            if self.a.agent.status != 'guiCommand':
                rospy.loginfo(self.a.id + ' setting GUI Goal Point...')
                self.a.lastGoalTime = rospy.get_rostime()
            self.a.setGoalPoint('guiCommand')


class GoHome(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['ReverseDrop', 'HumanInput']
        self.objectives = ['MaintainComms', 'ExtendComms', 'ReportArtifacts', 'Input']

    def evaluate(self):
        # Reporting and Maintaining comms primarily drive this behavior
        self.score = self.a.objectives['maintainComms'].weight + self.a.objectives['report'].weight

        # Extend comms also drives if the monitor says to reverse drop
        if self.a.reverseDrop:
            self.score += self.a.objectives['extendComms'].weight
        if self.a.guiBehavior == 'home':
            self.score += self.a.objectives['input'].weight

    def execute(self):
        reason = 'Home'
        if self.a.report:
            reason = 'Report'
            if self.a.agent.status != 'Report':
                rospy.loginfo(self.a.id + ' return to report...')
        if self.a.reverseDrop and not self.a.checkStatus('Regain comms deploy'):
            rospy.loginfo(self.a.id + ' reverse deploy mode')
            self.a.updateStatus('Regain comms deploy')
        self.a.setGoalPoint(reason)


class Stop(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['HumanInput']
        self.objectives = ['Input', 'BeSafe']

    def evaluate(self):
        self.score = self.a.objectives['beSafe'].weight
        if self.a.guiBehavior == 'stop':
            self.score = self.a.objectives['input'].weight

    def execute(self):
        self.a.stop()
