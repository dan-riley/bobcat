#!/usr/bin/env python
from __future__ import print_function
import rospy


class DefaultObjective():
    """ Default Objective object to extend for others """

    def __init__(self, agent, priority):
        self.a = agent
        self.priority = priority
        self.name = self.__class__.__name__
        self.monitors = []
        # Start our calculated weight at 0
        self.weight = 0
        # Set our baseline weight based on the priority level given
        # Alternatively, simply set self.initialWeight = priority (where priority is the weight)
        self.setInitialWeight()

    def setInitialWeight(self):
        # Set the range/boost used for weight calculations.  0.4/0.5 gives a range of 0.6 - 1.9.
        # Presets make ties difficult, and for highest priority objectives to always be fulfilled
        ranges = 0.4
        boost = 0.5

        if self.priority == 0:
            # For any objective that needs to be able to override no matter what, ie Input
            self.initialWeight = 1000
        elif self.a.numPriorities == 1:
            # Equal weight objectives
            self.initialWeight = 1
        else:
            # Mid-point of priorities where weight == 1
            mid = (self.a.numPriorities - 1) / 2
            # Normalize the given priorities around 1
            self.initialWeight = ((self.priority - 1 - mid) / mid) * (-ranges) + 1
            # Add additional weight to "higher than average" priorities
            if self.initialWeight > 1:
                self.initialWeight += boost

    def setWeight(self, weight = None):
        if weight is None:
            self.weight = self.initialWeight
        else:
            self.weight = weight

    def evaluate(self):
        self.setWeight()


class FindArtifacts(DefaultObjective):

    def __init__(self, agent, priority):
        DefaultObjective.__init__(self, agent, priority)
        self.monitors = ['ExploreToGoal']
        # The ExploreToGoal Monitor is associated, but the objective is always active


class ReportArtifacts(DefaultObjective):

    def __init__(self, agent, priority):
        DefaultObjective.__init__(self, agent, priority)
        self.monitors = ['Artifact']

    def evaluate(self):
        self.setWeight(0)
        if self.a.report:
            self.setWeight()


class Input(DefaultObjective):

    def __init__(self, agent, priority):
        DefaultObjective.__init__(self, agent, priority)
        self.monitors = ['HumanInput']

    def evaluate(self):
        self.setWeight(0)
        if self.a.guiBehavior:
            self.setWeight()


class MaintainComms(DefaultObjective):

    def __init__(self, agent, priority):
        DefaultObjective.__init__(self, agent, priority)
        self.monitors = ['Comms']

    def evaluate(self):
        self.setWeight(0)
        if not self.a.base.incomm:
            self.setWeight()


class ExtendComms(DefaultObjective):

    def __init__(self, agent, priority):
        DefaultObjective.__init__(self, agent, priority)
        self.monitors = ['Beacon', 'ReverseDrop']

    def evaluate(self):
        self.setWeight(0)
        if self.a.deployBeacon or self.a.reverseDrop:
            self.setWeight()


class BeSafe(DefaultObjective):

    def __init__(self, agent, priority):
        DefaultObjective.__init__(self, agent, priority)
        self.monitors = ['NearbyRobot']
        # Add additional weight to this objective since it's safety
        self.initialWeight += 1

    def evaluate(self):
        self.setWeight(0)
        if self.a.nearbyRobot:
            self.setWeight()
