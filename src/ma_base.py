#!/usr/bin/env python
from __future__ import print_function
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from marble_multi_agent.msg import AgentArtifact
from marble_multi_agent.msg import AgentReset

from multi_agent import MultiAgent


class MABase(MultiAgent):
    """ Initialize a multi-agent base station node """

    def __init__(self):
        # Get all of the parent class variables
        MultiAgent.__init__(self)
        # Force the monitors on for Base type
        self.useMonitor = True

        for nid in self.neighbors:
            self.addGUIMonitor(nid)

        # Listen for new robots to be added to the system
        self.monitor['addRobot'] = rospy.Subscriber('add_robot', String, self.AddRobotReceiver)

        self.monitor['beacons'] = rospy.Publisher('mbeacons', Marker, queue_size=10)
        self.mbeacon = Marker()
        self.mbeacon.header.frame_id = 'world'
        self.mbeacon.type = self.mbeacon.CUBE_LIST
        self.mbeacon.action = self.mbeacon.ADD
        self.mbeacon.scale.x = 1.0
        self.mbeacon.scale.y = 1.0
        self.mbeacon.scale.z = 1.0
        self.mbeacon.color.a = 1.0
        self.mbeacon.color.r = 1.0
        self.mbeacon.color.g = 1.0
        self.mbeacon.color.b = 1.0

        self.monitor['artifacts'] = rospy.Publisher('martifacts', MarkerArray, queue_size=10)
        self.martifact = MarkerArray()

        self.commListen = True

    def addGUIMonitor(self, nid):
        topic = 'neighbors/' + nid + '/'
        # Monitor GUI commands to send over the network
        self.monitor[nid]['guiTaskName'] = \
            rospy.Subscriber(topic + 'guiTaskName', String, self.GuiTaskNameReceiver, nid)
        self.monitor[nid]['guiTaskValue'] = \
            rospy.Subscriber(topic + 'guiTaskValue', String, self.GuiTaskValueReceiver, nid)
        self.monitor[nid]['guiGoalPoint'] = \
            rospy.Subscriber(topic + 'guiGoalPoint', Pose, self.GuiGoalReceiver, nid)
        self.monitor[nid]['guiReset'] = \
            rospy.Subscriber(topic + 'guiReset', AgentReset, self.GuiResetReceiver, nid)

    def buildArtifactMarkers(self):
        i = 0
        self.martifact.markers = []
        for artifact in self.artifacts.values():

            martifact = Marker()
            martifact.header = artifact.artifact.header
            martifact.header.frame_id = 'world'
            martifact.id = i
            martifact.type = martifact.TEXT_VIEW_FACING
            martifact.action = martifact.ADD

            # Highlight new artifacts, and check if it's now old
            if artifact.new:
                martifact.scale.x = 10.0
                martifact.scale.y = 10.0
                martifact.scale.z = 10.0

                try:
                    if artifact.firstSeen < rospy.get_rostime() - rospy.Duration(30):
                        artifact.new = False
                except TypeError:
                    continue
            else:
                martifact.scale.x = 1.0
                martifact.scale.y = 1.0
                martifact.scale.z = 1.0

            martifact.color.a = 1.0
            martifact.color.r = 1.0
            martifact.color.g = 1.0
            martifact.color.b = 1.0
            martifact.pose.position = artifact.artifact.position
            martifact.text = artifact.artifact.obj_class

            self.martifact.markers.append(martifact)
            i = i + 1

    def publishNeighbors(self):
        self.mbeacon.points = []
        for beacon in self.beacons.values():
            if beacon.active:
                self.mbeacon.points.append(beacon.pos)
        self.monitor['beacons'].publish(self.mbeacon)

        self.buildArtifactMarkers()
        self.monitor['artifacts'].publish(self.martifact)

    def AddRobotReceiver(self, data):
        # Add a new robot to the system, which will propogate to any other agents in comms
        if data.data not in self.neighbors:
            self.addNeighbor(data.data, 'robot')
            self.addGUIMonitor(data.data)

    def GuiTaskNameReceiver(self, data, nid):
        self.neighbors[nid].guiStamp = rospy.get_rostime()
        self.neighbors[nid].guiTaskName = data.data

    def GuiTaskValueReceiver(self, data, nid):
        self.neighbors[nid].guiStamp = rospy.get_rostime()
        self.neighbors[nid].guiTaskValue = data.data

    def GuiGoalReceiver(self, data, nid):
        # Don't accept a 0,0 goal due to GUI errors
        if data.position.x or data.position.y:
            self.neighbors[nid].guiStamp = rospy.get_rostime()
            self.neighbors[nid].guiGoalPoint.header.frame_id = 'world'
            self.neighbors[nid].guiGoalPoint.header.seq += 1
            self.neighbors[nid].guiGoalPoint.pose = data
            self.neighbors[nid].guiTaskName = 'task'
            self.neighbors[nid].guiTaskValue = 'Goal'

    def GuiResetReceiver(self, data, nid):
        if data.agent == nid:
            # If the data is only going to the robot then we just add here and don't process
            if not data.base and data.robots and data.stamp > self.neighbors[nid].resetStamp:
                self.neighbors[nid].guiStamp = rospy.get_rostime()
                self.neighbors[nid].reset = data
            else:
                self.resetDataCheck(data)

    def buildBaseArtifacts(self):
        # Set all of the current base station data
        self.base.baseStamp = rospy.get_rostime()
        self.base.baseArtifacts = []
        for neighbor in self.neighbors.values():
            agent = AgentArtifact()
            agent.id = neighbor.id
            agent.lastArtifact = neighbor.lastArtifact
            self.base.baseArtifacts.append(agent)

    def run(self):
        if self.updateArtifacts():
            self.buildBaseArtifacts()
        self.publishNeighbors()

        return True
