#!/usr/bin/env python
from __future__ import print_function
import rospy
import copy

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from marble_artifact_detection_msgs.msg import Artifact
from bobcat.msg import AgentArtifact
from bobcat.msg import AgentReset
from bobcat.msg import ArtifactScore

from BOBCAT import BOBCAT, getDist2D


class BCBase(BOBCAT):
    """ Initialize a multi-agent base station node """

    def __init__(self):
        # Get all of the parent class variables
        BOBCAT.__init__(self)
        # Force the monitors on for Base type
        self.useMonitor = True
        # Distance to fuse artifacts within.  May want smaller to account for missed score reports.
        self.fuseDist = rospy.get_param('bobcat/fuseDist', 3.0)
        # Storage for fused artifacts and reporting
        self.fusedArtifacts = {}
        self.fused_pub = rospy.Publisher('artifact_report', Artifact, queue_size=10)
        self.score_sub = rospy.Subscriber('artifact_score', ArtifactScore, self.GetArtifactScore)

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

    def GetArtifactScore(self, data):
        self.fusedArtifacts[data.id].score = data.score
        self.fusedArtifacts[data.id].reported = True

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

        self.artifactsUpdated = False

    def fuseArtifact(self, artifact):
        fuse = False
        rem = []
        for fartifact in self.fusedArtifacts.values():
            if (artifact.artifact.obj_class == fartifact.artifact.obj_class and
                    getDist2D(artifact.artifact.position, fartifact.artifact.position) < self.fuseDist):
                fuse = True
                if artifact.id not in fartifact.originals:
                    fartifact.originals[artifact.id] = artifact

                # Get the averages for position and probability
                x = 0
                y = 0
                z = 0
                obj_prob = 0
                for artifact in fartifact.originals.values():
                    x += artifact.artifact.position.x
                    y += artifact.artifact.position.y
                    z += artifact.artifact.position.z
                    obj_prob += artifact.artifact.obj_prob

                length = len(fartifact.originals)
                new_fartifact = copy.deepcopy(fartifact)
                new_fartifact.artifact.position.x = x / length
                new_fartifact.artifact.position.y = y / length
                new_fartifact.artifact.position.z = z / length
                new_fartifact.artifact.obj_prob = obj_prob / length

                # Update the id
                new_fartifact.id += '_' + artifact.id
                new_fartifact.artifact.artifact_id = new_fartifact.id
                # For now don't reset report flag.  If it was reported don't waste an attempt,
                # particularly if fuseDist is small!  If ack's become more reliable maybe change
                # If we didn't score with this artifact already, then make sure we try to report
                # if not new_fartifact.score:
                #     new_fartifact.reported = False

                # Save the artifact
                self.fusedArtifacts[new_fartifact.id] = new_fartifact
                # Identify old fused artifacts to remove
                rem.append(fartifact.id)

        # Remove old fused artifacts that aren't needed anymore
        for rid in rem:
            del self.fusedArtifacts[rid]

        # This artifact hasn't been fused yet, so add it
        if not fuse:
            self.fusedArtifacts[artifact.id] = copy.deepcopy(artifact)
            self.fusedArtifacts[artifact.id].originals[artifact.id] = artifact

    def reportArtifacts(self):
        for artifact in self.fusedArtifacts.values():
            if not artifact.reported:
                self.fused_pub.publish(artifact.artifact)

    def run(self):
        self.updateArtifacts()
        if self.artifactsUpdated:
            self.buildBaseArtifacts()
        self.reportArtifacts()
        self.publishNeighbors()

        return True
