#!/usr/bin/env python
from __future__ import print_function
import hashlib
import rospy

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from marble_artifact_detection_msgs.msg import ArtifactArray
from marble_artifact_detection_msgs.msg import ArtifactImg
from bobcat.msg import AgentReset
from bobcat.msg import BeaconArray
from bobcat.msg import Goal
from bobcat.msg import GoalCompressed
from bobcat.msg import GoalArray
from marble_mapping.msg import OctomapArray


class Agent(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id, parent_id, agent_type, report_images):
        self.id = agent_id
        self.pid = parent_id
        self.cid = ''
        self.type = agent_type
        self.reset = AgentReset()
        self.reportImages = report_images
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
        self.incomm = True
        self.simcomm = True
        self.initialize()
        self.initializeMaps()

        # Initialize our empty path so guidance controller can ignore it
        if (self.id == self.pid):
            self.explorePath.header.frame_id = 'starting'

    def initialize(self, resetTime=rospy.Time()):
        self.status = ''
        self.battery = 0
        self.guiStamp = rospy.get_rostime()
        self.guiTaskName = ''
        self.guiTaskValue = ''
        self.guiGoalPoint = PoseStamped()
        self.guiAccept = False
        self.guiGoalAccept = False
        self.odometry = Odometry()
        self.exploreGoal = PoseStamped()
        self.explorePath = Path()
        self.exploreGoal.header.frame_id = 'world'
        self.explorePath.header.frame_id = 'world'
        self.goal = Goal()
        self.goalCompressed = GoalCompressed()
        self.goals = GoalArray()
        self.poseGraph = Path()
        self.poseGraphCompressed = []
        self.latestPoseGraph = 0
        self.latestPoseGraphAvailable = 0
        self.atnode = Bool()
        self.commBeacons = BeaconArray()
        self.newArtifacts = ArtifactArray()
        self.checkArtifacts = ArtifactArray()
        self.images = []
        self.missingImages = []
        self.lastArtifact = ''
        self.lastArtifactPub = rospy.Time();
        self.resetStamp = resetTime
        if resetTime:
            self.resetAgent = True
        else:
            self.resetAgent = False

    def initializeMaps(self, numDiffs=0, diffClear=False):
        self.mapDiffs = OctomapArray()
        self.mapDiffs.owner = self.id
        self.updateMapDiffs = False
        self.numDiffs = numDiffs
        self.missingDiffs = []
        self.diffClear = diffClear

    def updateCommon(self, neighbor):
        self.status = neighbor.status
        self.battery = neighbor.battery
        self.latestPoseGraph = neighbor.latestPoseGraph

        # Empty artifacts are sent most of the time so only store if not
        if neighbor.newArtifacts.artifacts:
            self.newArtifacts = neighbor.newArtifacts

        # Convert PoseStamped to Odometry
        self.odometry.header = neighbor.odometry.header
        self.odometry.pose.pose = neighbor.odometry.pose

        # Extract the goal and convert from compressed to uncompressed
        self.goalCompressed = neighbor.goal
        self.goal.cost = neighbor.goal.cost
        self.goal.path = self.decompressPath(neighbor.goal.path)
        # Goal is the last point in the path
        if len(self.goal.path.poses):
            self.goal.pose = self.goal.path.poses[-1]

        # Update missing diffs if the neighbor said there are new ones
        if not self.diffClear and neighbor.numDiffs > self.numDiffs and not self.reset.ignore:
            for i in range(self.numDiffs, neighbor.numDiffs):
                if i not in self.missingDiffs:
                    self.missingDiffs.append(i)
            self.numDiffs = neighbor.numDiffs

        # Identify new images available for request
        for artifact in neighbor.newArtifacts.artifacts:
            if (artifact.image_data.format != 'empty' and
                    artifact.artifact_id not in self.images and
                    artifact.artifact_id not in self.missingImages):
                # Images we know about so we don't re-mark them for download
                self.images.append(artifact.artifact_id)
                # Images we haven't received yet
                self.missingImages.append(artifact.artifact_id)

        # Ignore the GUI message if it's old
        if neighbor.guiStamp.data > self.guiStamp:
            self.guiUpdate(neighbor)

    def update(self, neighbor, updater=False):
        # If this agent has been reset, don't accept messages until confirmed it has reset
        if self.resetAgent:
            if neighbor.reset.stamp < self.resetStamp:
                return
            else:
                self.resetAgent = False

        # Update parameters depending on if we're talking directly or not
        if updater:
            self.commBeacons = neighbor.commBeacons
            self.cid = updater
            self.lastMessage = neighbor.header.stamp
            self.lastDirectMessage = rospy.get_rostime()
            self.incomm = True
            self.updateCommon(neighbor)
        else:
            self.updateCommon(neighbor)
            self.cid = neighbor.cid
            self.incomm = False
            self.lastMessage = neighbor.lastMessage.data

    def guiUpdate(self, neighbor):
        self.guiStamp = neighbor.guiStamp.data

        if neighbor.guiTaskName and neighbor.guiTaskValue:
            self.guiTaskName = neighbor.guiTaskName
            self.guiTaskValue = neighbor.guiTaskValue
            self.guiAccept = True

        # Accept goal point if it's updated
        if neighbor.guiGoalPoint.header.seq > self.guiGoalPoint.header.seq:
            self.guiGoalPoint = neighbor.guiGoalPoint

        # Only accept reset if it's newer than the last one for this agent
        if neighbor.reset.stamp > self.reset.stamp:
            self.reset = neighbor.reset

    def addArtifact(self, artifact):
        self.checkArtifacts.artifacts.append(artifact)
        self.checkArtifacts.owner = self.id
        self.checkArtifacts.num_artifacts += 1

    def updateHash(self):
        # If set, ignore images when checking if Base has received new artifacts
        if not self.reportImages:
            for artifact in self.checkArtifacts.artifacts:
                artifact.image_data.data = []
        artifactString = repr(self.checkArtifacts.artifacts).encode('utf-8')
        self.lastArtifact = hashlib.md5(artifactString).hexdigest()

    def decompressPath(self, cpath):
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = "world"
        i = 0
        while i < len(cpath):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = rospy.get_rostime()
            pose.pose.position.x = 0.1 * cpath[i]
            i += 1
            pose.pose.position.y = 0.1 * cpath[i]
            i += 1
            pose.pose.position.z = 0.1 * cpath[i]
            i += 1
            path.poses.append(pose)

        return path


class Base(object):
    """ Data structure to hold pertinent information about the base station """

    def __init__(self):
        self.baseStamp = rospy.get_rostime()
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
        self.lastArtifact = ''
        self.incomm = True
        self.simcomm = True
        self.baseArtifacts = []
        self.commBeacons = BeaconArray()

    def update(self, neighbor):
        self.lastMessage = neighbor.header.stamp
        self.lastDirectMessage = rospy.get_rostime()
        self.commBeacons = neighbor.commBeacons

    def updateArtifacts(self, agent_id, neighbor):
        self.baseStamp = neighbor.baseStamp.data
        self.baseArtifacts = neighbor.baseArtifacts
        for agent in neighbor.baseArtifacts:
            if agent.id == agent_id:
                self.lastArtifact = agent.lastArtifact
                break

    def resetArtifact(self, agent_id):
        for agent in self.baseArtifacts:
            if agent.id == agent_id:
                self.lastArtifact = ''
                break


class BeaconObj(object):
    """ Data structure to hold pertinent information about beacons """

    def __init__(self, agent, owner):
        self.id = agent
        self.owner = owner
        self.pos = Point()
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
        self.incomm = False
        self.simcomm = False
        self.active = False

    def update(self, neighbor):
        self.lastMessage = neighbor.header.stamp
        self.lastDirectMessage = rospy.get_rostime()


class ArtifactReport:
    """
    Internal artifact structure to track reporting.
    Holds full artifact message so neighbors can be fused.
    """

    def __init__(self, agent_id, artifact, sendImages):
        self.id = artifact.artifact_id
        self.agent_id = agent_id
        self.artifact = artifact
        self.reported = False
        self.score = 0
        self.new = True
        self.firstSeen = rospy.get_rostime()
        self.lastPublished = rospy.Time()
        self.originals = {}

        # Mark empty data so we receiver doesn't try to request it
        if not artifact.image_data.data or not sendImages:
            artifact.image_data.format = 'empty'
        # Save image so we can send it via DM
        self.image = ArtifactImg()
        if sendImages:
            self.image.artifact_id = artifact.artifact_id
            self.image.artifact_img = artifact.image_data
