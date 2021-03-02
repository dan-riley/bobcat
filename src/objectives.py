#!/usr/bin/env python
from __future__ import print_function
import rospy


class DefaultObjective():
    """ Default Objective object to extend for others """

    def __init__(self, agent):
        self.a = agent
        # Weight for this objective.  Usually 1 for simple objectives.
        self.initialWeight = 1
        self.weight = self.initialWeight
        # Priority flag for objectives deemed important to mission success.
        # Enabling using setPriority doubles the multiplier
        self.priority = False
        self.multiplier = 1

    def setPriority(self):
        self.priority = True
        self.multiplier = 2

    def setWeight(self, weight = None):
        if weight is None:
            self.weight = self.initialWeight * self.multiplier
        else:
            self.weight = weight

    def evaluate(self):
        self.setWeight()


class Explore(DefaultObjective):
    pass


class ReportArtifacts(DefaultObjective):

    def evaluate(self):
        self.setWeight(0)
        if self.a.report:
            self.setWeight()


class Input(DefaultObjective):

    def evaluate(self):
        self.setWeight(0)
        if self.a.guiBehavior:
            self.setWeight()


class MaintainComms(DefaultObjective):

    def __init__(self, agent):
        DefaultObjective.__init__(self, agent)
        self.initialWeight = 0.7

    def evaluate(self):
        self.setWeight(0)
        if not self.a.base.incomm:
            self.setWeight()


class ExtendComms(DefaultObjective):

    def evaluate(self):
        self.setWeight(0)
        if self.a.deployBeacon or self.a.reverseDrop:
            self.setWeight()
